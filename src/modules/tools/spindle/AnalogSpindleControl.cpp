/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "libs/Pin.h"
#include "AnalogSpindleControl.h"
#include "Config.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "StreamOutputPool.h"
#include "SlowTicker.h"
#include "PublicDataRequest.h"
#include "SpindlePublicAccess.h"
#include "PwmOut.h"
#include "InterruptIn.h"
#include "port_api.h"
#include "system_LPC17xx.h"
#include "us_ticker_api.h"

#define spindle_checksum                    CHECKSUM("spindle")
#define spindle_max_rpm_checksum            CHECKSUM("max_rpm")
#define spindle_min_rpm_checksum            CHECKSUM("min_rpm")
#define spindle_pwm_pin_checksum            CHECKSUM("pwm_pin")
#define spindle_pwm_period_checksum         CHECKSUM("pwm_period")
#define spindle_switch_on_pin_checksum      CHECKSUM("switch_on_pin")
#define spindle_feedback_pin_checksum       CHECKSUM("feedback_pin")
#define spindle_pulses_per_rev_checksum     CHECKSUM("pulses_per_rev")
#define spindle_control_smoothing_checksum  CHECKSUM("control_smoothing")
#define spindle_acc_ratio_checksum          CHECKSUM("acc_ratio")
#define spindle_alarm_pin_checksum          CHECKSUM("alarm_pin")

#define UPDATE_FREQ 100

void AnalogSpindleControl::on_module_loaded()
{
    spindle_on = false;
    target_rpm = 0;
    current_rpm = 0;
    current_pwm_value = 0;
    factor = 100;
    irq_count = 0;
    last_rev_time = 0;
    rev_time = 0;
    time_since_update = 0;

    min_rpm = THEKERNEL->config->value(spindle_checksum, spindle_min_rpm_checksum)->by_default(100)->as_int();
    max_rpm = THEKERNEL->config->value(spindle_checksum, spindle_max_rpm_checksum)->by_default(5000)->as_int();

    pulses_per_rev = THEKERNEL->config->value(spindle_checksum, spindle_pulses_per_rev_checksum)->by_default(1.0f)->as_number();
    acc_ratio = THEKERNEL->config->value(spindle_checksum, spindle_acc_ratio_checksum)->by_default(1.0f)->as_number();
    alarm_pin.from_string(THEKERNEL->config->value(spindle_checksum, spindle_alarm_pin_checksum)->by_default("nc")->as_string())->as_input();

    float smoothing_time = THEKERNEL->config->value(spindle_checksum, spindle_control_smoothing_checksum)->by_default(0.1f)->as_number();
    if (smoothing_time * UPDATE_FREQ < 1.0f)
        smoothing_decay = 1.0f;
    else
        smoothing_decay = 1.0f / (UPDATE_FREQ * smoothing_time);

    {
        Pin *smoothie_pin = new Pin();
        smoothie_pin->from_string(THEKERNEL->config->value(spindle_checksum, spindle_pwm_pin_checksum)->by_default("nc")->as_string());
        pwm_pin = smoothie_pin->as_output()->hardware_pwm();
        output_inverted = smoothie_pin->is_inverting();
        delete smoothie_pin;
    }
    if (pwm_pin == NULL)
    {
        THEKERNEL->streams->printf("Error: Spindle PWM pin must be P2.0-2.5 or other PWM pin\n");
        delete this;
        return;
    }

    int period = THEKERNEL->config->value(spindle_checksum, spindle_pwm_period_checksum)->by_default(1000)->as_int();
    THEKERNEL->Spindle_period_us = period;
    pwm_pin->period_us(period);
    pwm_pin->write(output_inverted ? 1 : 0);

    {
        Pin *smoothie_pin = new Pin();
        smoothie_pin->from_string(THEKERNEL->config->value(spindle_checksum, spindle_feedback_pin_checksum)->by_default("nc")->as_string());
        feedback_pin = NULL;
        if (smoothie_pin->connected()) {
            smoothie_pin->as_input();
            if (smoothie_pin->port_number == 0 || smoothie_pin->port_number == 2) {
                PinName pinname = port_pin((PortName)smoothie_pin->port_number, smoothie_pin->pin);
                feedback_pin = new mbed::InterruptIn(pinname);
                feedback_pin->rise(this, &AnalogSpindleControl::on_pin_rise);
                NVIC_SetPriority(EINT3_IRQn, 16);
            } else {
                THEKERNEL->streams->printf("Error: Spindle feedback pin has to be on P0 or P2.\n");
                delete smoothie_pin;
                delete this;
                return;
            }
        }
        delete smoothie_pin;
    }

    std::string switch_on_pin = THEKERNEL->config->value(spindle_checksum, spindle_switch_on_pin_checksum)->by_default("nc")->as_string();
    switch_on = NULL;
    if(switch_on_pin.compare("nc") != 0) {
        switch_on = new Pin();
        switch_on->from_string(switch_on_pin)->as_output()->set(false);
    }

    THEKERNEL->slow_ticker->attach(UPDATE_FREQ, this, &AnalogSpindleControl::on_update_speed);
}

void AnalogSpindleControl::on_pin_rise()
{
    if (irq_count >= pulses_per_rev) {
        irq_count = 0;
        uint32_t timestamp = us_ticker_read();
        rev_time = timestamp - last_rev_time;
        last_rev_time = timestamp;
        time_since_update = 0;
    }
    irq_count++;
}

uint32_t AnalogSpindleControl::on_update_speed(uint32_t dummy)
{
    if (feedback_pin != NULL) {
        if (++time_since_update > UPDATE_FREQ)
        {
            current_rpm = 0;
        }
        else
        {
            uint32_t t = rev_time;
            if (t > (uint32_t)(2000 * acc_ratio))
            {
                float new_rpm = 1000000 * acc_ratio * 60.0f / t;
                current_rpm = smoothing_decay * new_rpm + (1.0f - smoothing_decay) * current_rpm;
            }
        }
    }
    else
    {
        if (!spindle_on || target_rpm <= 0)
            current_rpm = 0;
        else
        {
            float duty = pwm_pin->read();
            current_rpm = max_rpm * duty;
        }
    }
    return 0;
}

bool AnalogSpindleControl::get_alarm(void)
{
    uint32_t debounce = 0;
    while (this->alarm_pin.get()) {
        if ( ++debounce >= 10 ) {
            return true;
        }
    }
    return false;
}

void AnalogSpindleControl::on_idle(void *argument)
{
    (void)argument;
    if(THEKERNEL->is_halted()) return;
    if (this->get_alarm()) {
        THEKERNEL->streams->printf("ALARM: Spindle alarm triggered -  power off/on required\n");
        THEKERNEL->set_halt_reason(SPINDLE_ALARM);
        THEKERNEL->call_event(ON_HALT, nullptr);
    }
}

void AnalogSpindleControl::apply_pwm_from_targets()
{
    if (!spindle_on || target_rpm <= 0) {
        update_pwm(0);
        return;
    }
    float cmd = target_rpm * (factor / 100.0f);
    if (cmd > (float)max_rpm) cmd = (float)max_rpm;
    update_pwm(1.0f / max_rpm * cmd);
}

void AnalogSpindleControl::turn_on()
{
    if(switch_on != NULL)
        switch_on->set(true);
    spindle_on = true;
    THEKERNEL->spindleon = true;
    apply_pwm_from_targets();
}

void AnalogSpindleControl::turn_off()
{
    if(switch_on != NULL)
        switch_on->set(false);
    spindle_on = false;
    THEKERNEL->spindleon = false;
    update_pwm(0);
}

void AnalogSpindleControl::set_speed(int rpm)
{
    float tr = rpm;
    if(rpm < 0) {
        tr = 0;
    } else if (rpm > max_rpm) {
        tr = max_rpm;
    } else if (rpm > 0 && rpm < min_rpm){
        tr = min_rpm;
    }
    target_rpm = tr;
    apply_pwm_from_targets();
}

void AnalogSpindleControl::report_speed()
{
    THEKERNEL->streams->printf("State: %s, Current RPM: %5.0f  Target RPM: %5.0f  PWM value: %5.3f\n",
            spindle_on ? "on" : "off", current_rpm, target_rpm, current_pwm_value);
}

void AnalogSpindleControl::update_pwm(float value)
{
    current_pwm_value = value;
    if(output_inverted)
        pwm_pin->write(1.0f - value);
    else
        pwm_pin->write(value);
}

void AnalogSpindleControl::set_factor(float new_factor)
{
    factor = new_factor;
    apply_pwm_from_targets();
}

void AnalogSpindleControl::on_get_public_data(void* argument)
{
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);
    if(!pdr->starts_with(pwm_spindle_control_checksum)) return;
    if(pdr->second_element_is(get_spindle_status_checksum)) {
        struct spindle_status *t= static_cast<spindle_status*>(pdr->get_data_ptr());
        t->state = this->spindle_on;
        t->current_rpm = this->current_rpm;
        t->target_rpm = this->target_rpm;
        t->current_pwm_value = this->current_pwm_value;
        t->factor= this->factor;
        pdr->set_taken();
    }
}

void AnalogSpindleControl::on_set_public_data(void* argument)
{
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);
    if(!pdr->starts_with(pwm_spindle_control_checksum)){
        return;
    }
    if(pdr->second_element_is(get_spindle_status_checksum)) {
        struct spindle_status *t= static_cast<spindle_status*>(pdr->get_data_ptr());
        this->set_factor(t->factor);
        pdr->set_taken();
        return;
    }
    if(pdr->second_element_is(turn_off_spindle_checksum)) {
        this->turn_off();
        pdr->set_taken();
    }
}
