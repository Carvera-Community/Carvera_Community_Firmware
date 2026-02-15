/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "PWMSpindleControl.h"
#include "Config.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "StreamOutputPool.h"
#include "SlowTicker.h"
#include "Conveyor.h"
#include "system_LPC17xx.h"
#include "PublicDataRequest.h"
#include "SpindlePublicAccess.h"
#include "utils.h"

#include "libs/Pin.h"
#include "Gcode.h"
#include "InterruptIn.h"
#include "PwmOut.h"
#include "port_api.h"
#include "us_ticker_api.h"

#define spindle_checksum                    CHECKSUM("spindle")
#define spindle_pwm_pin_checksum            CHECKSUM("pwm_pin")
#define spindle_pwm_period_checksum         CHECKSUM("pwm_period")
#define spindle_max_pwm_checksum            CHECKSUM("max_pwm")
#define spindle_feedback_pin_checksum       CHECKSUM("feedback_pin")
#define spindle_pulses_per_rev_checksum     CHECKSUM("pulses_per_rev")
#define spindle_default_rpm_checksum        CHECKSUM("default_rpm")
#define spindle_control_P_checksum          CHECKSUM("control_P")
#define spindle_control_I_checksum          CHECKSUM("control_I")
#define spindle_control_D_checksum          CHECKSUM("control_D")
#define spindle_control_smoothing_checksum  CHECKSUM("control_smoothing")
#define spindle_delay_s_checksum			CHECKSUM("delay_s")
#define spindle_acc_ratio_checksum			CHECKSUM("acc_ratio")
#define spindle_alarm_pin_checksum			CHECKSUM("alarm_pin")
#define spindle_stall_s_checksum			CHECKSUM("stall_s")
#define spindle_stall_count_rpm_checksum	CHECKSUM("stall_count_rpm")
#define spindle_stall_alarm_rpm_checksum	CHECKSUM("stall_alarm_rpm")

#define UPDATE_FREQ 100

PWMSpindleControl::PWMSpindleControl()
{
}

void PWMSpindleControl::on_module_loaded()
{
    last_time = 0;
    last_edge = 0;
    current_rpm = 0;
    current_I_value = 0;
    current_pwm_value = 0;
    time_since_update = 0;
    stall_timer = 0;
    
    spindle_on = false;
    
    factor = 100;

    pulses_per_rev = THEKERNEL->config->value(spindle_checksum, spindle_pulses_per_rev_checksum)->by_default(1.0f)->as_number();
    target_rpm = THEKERNEL->config->value(spindle_checksum, spindle_default_rpm_checksum)->by_default(10000.0f)->as_number();
    control_P_term = THEKERNEL->config->value(spindle_checksum, spindle_control_P_checksum)->by_default(0.0001f)->as_number();
    control_I_term = THEKERNEL->config->value(spindle_checksum, spindle_control_I_checksum)->by_default(0.0001f)->as_number();
    control_D_term = THEKERNEL->config->value(spindle_checksum, spindle_control_D_checksum)->by_default(0.0001f)->as_number();

    delay_s        = THEKERNEL->config->value(spindle_checksum, spindle_delay_s_checksum)->by_default(3)->as_number();
    stall_s        = THEKERNEL->config->value(spindle_checksum, spindle_stall_s_checksum)->by_default(100)->as_number();
    stall_count_rpm = THEKERNEL->config->value(spindle_checksum, spindle_stall_count_rpm_checksum)->by_default(8000)->as_number();
    stall_alarm_rpm = THEKERNEL->config->value(spindle_checksum, spindle_stall_alarm_rpm_checksum)->by_default(5000)->as_number();
    acc_ratio      = THEKERNEL->config->value(spindle_checksum, spindle_acc_ratio_checksum)->by_default(1.0f)->as_number();
    alarm_pin.from_string(THEKERNEL->config->value(spindle_checksum, spindle_alarm_pin_checksum)->by_default("nc")->as_string())->as_input();

    // Smoothing value is low pass filter time constant in seconds.
    float smoothing_time = THEKERNEL->config->value(spindle_checksum, spindle_control_smoothing_checksum)->by_default(0.1f)->as_number();
    if (smoothing_time * UPDATE_FREQ < 1.0f)
        smoothing_decay = 1.0f;
    else
        smoothing_decay = 1.0f / (UPDATE_FREQ * smoothing_time);

    // Get the pin for hardware pwm
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

    max_pwm = THEKERNEL->config->value(spindle_checksum, spindle_max_pwm_checksum)->by_default(1.0f)->as_number();
    if(CARVERA_AIR == THEKERNEL->factory_set->MachineModel)
    {
    	max_pwm = 0.9;
    }
    
    int period = THEKERNEL->config->value(spindle_checksum, spindle_pwm_period_checksum)->by_default(1000)->as_int();
    THEKERNEL->Spindle_period_us = period;
    pwm_pin->period_us(period);
    pwm_pin->write(output_inverted ? 1 : 0);

    // Get the pin for interrupt
    {
        Pin *smoothie_pin = new Pin();
        smoothie_pin->from_string(THEKERNEL->config->value(spindle_checksum, spindle_feedback_pin_checksum)->by_default("nc")->as_string());
        smoothie_pin->as_input();
        if (smoothie_pin->port_number == 0 || smoothie_pin->port_number == 2) {
            PinName pinname = port_pin((PortName)smoothie_pin->port_number, smoothie_pin->pin);
            feedback_pin = new mbed::InterruptIn(pinname);
            feedback_pin->rise(this, &PWMSpindleControl::on_pin_rise);
            NVIC_SetPriority(EINT3_IRQn, 16);
        } else {
            THEKERNEL->streams->printf("Error: Spindle feedback pin has to be on P0 or P2.\n");
            delete this;
            return;
        }
        delete smoothie_pin;
    }
    
    THEKERNEL->slow_ticker->attach(UPDATE_FREQ, this, &PWMSpindleControl::on_update_speed);
}

void PWMSpindleControl::on_pin_rise()
{
	if (irq_count >= pulses_per_rev) {
		irq_count = 0;
		rev_count ++;
		uint32_t timestamp = us_ticker_read();
		rev_time = timestamp - last_rev_time;
		last_rev_time = timestamp;
		time_since_update = 0;
	}
	irq_count ++;
}

uint32_t PWMSpindleControl::on_update_speed(uint32_t dummy)
{
    // If we don't get any interrupts for 1 second, set current RPM to 0
    if (++time_since_update > UPDATE_FREQ)
    {
    	current_rpm = 0;
    }
    else{    // Calculate current RPM

	    uint32_t t = rev_time;
	    if (t > 2000 * acc_ratio ) //RPM < 30000
	    {	
	        float new_rpm = 1000000 * acc_ratio * 60.0f / t;
	        current_rpm = smoothing_decay * new_rpm + (1.0f - smoothing_decay) * current_rpm;
	    }
	}
	if(CARVERA_AIR == THEKERNEL->factory_set->MachineModel)
    {
		if( fabs(target_rpm - last_rpm) > 500 )
		{
			target_change_count = 0;
			last_rpm = target_rpm;
		}
	}
    if (spindle_on) {
    	if (update_count > UPDATE_FREQ / 5) {
    		update_count = 0;
            float error = target_rpm * (factor / 100) - current_rpm;
            float acc_pwm = control_P_term * error;
            
            if(CARVERA_AIR == THEKERNEL->factory_set->MachineModel)
            {
            	if(target_change_count>UPDATE_FREQ*5)	//when speed changed time < 7s,we didn't use D_ter,to rapid speed up/down
	            {
	            	acc_pwm += control_D_term * (error - prev_error);
	            	target_change_count = UPDATE_FREQ*5;
            	}
            }
            float new_pwm = current_pwm_value + acc_pwm;
            new_pwm = confine(new_pwm, 0.0f, max_pwm);

            prev_error = error;
            current_pwm_value = new_pwm;
    	}
    	update_count ++;
    	if(CARVERA_AIR == THEKERNEL->factory_set->MachineModel)
        {
			target_change_count ++;
		}

		if (current_pwm_value > max_pwm) {
			current_pwm_value = max_pwm;
		}
    } else {
        current_I_value = 0;
        current_pwm_value = 0;
    }

    if (output_inverted)
        pwm_pin->write(1.0f - current_pwm_value);
    else
        pwm_pin->write(current_pwm_value);
    
    return 0;
}

void PWMSpindleControl::turn_on() {
    spindle_on = true;
    THEKERNEL->spindleon = true;
    if (delay_s > 0) {
        char buf[80];
        size_t n = snprintf(buf, sizeof(buf), "G4P%d", delay_s);
        if(n > sizeof(buf)) n= sizeof(buf);
        string g(buf, n);
        Gcode gcode(g, &(StreamOutput::NullStream));
        THEKERNEL->call_event(ON_GCODE_RECEIVED, &gcode);
    }
}

void PWMSpindleControl::turn_off() {
    spindle_on = false;
    THEKERNEL->spindleon = false;
    if (delay_s > 0) {
        char buf[80];
        size_t n = snprintf(buf, sizeof(buf), "G4P%d", delay_s);
        if(n > sizeof(buf)) n= sizeof(buf);
        string g(buf, n);
        Gcode gcode(g, &(StreamOutput::NullStream));
        THEKERNEL->call_event(ON_GCODE_RECEIVED, &gcode);
    }
}


void PWMSpindleControl::set_speed(int rpm) {
    target_rpm = rpm;
}


void PWMSpindleControl::report_speed() {
    THEKERNEL->streams->printf("State: %s, Current RPM: %5.0f  Target RPM: %5.0f  PWM value: %5.3f\n",
    			spindle_on ? "on" : "off", current_rpm, target_rpm, current_pwm_value);
}


void PWMSpindleControl::set_p_term(float p) {
    control_P_term = p;
}


void PWMSpindleControl::set_i_term(float i) {
    control_I_term = i;
}


void PWMSpindleControl::set_d_term(float d) {
    control_D_term = d;
}


void PWMSpindleControl::report_settings() {
    THEKERNEL->streams->printf("P: %0.6f I: %0.6f D: %0.6f\n",
                               control_P_term, control_I_term, control_D_term);
}

void PWMSpindleControl::set_factor(float new_factor) {
	factor = new_factor;
}

// returns spindle status
void PWMSpindleControl::on_get_public_data(void* argument)
{
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);
    if(!pdr->starts_with(pwm_spindle_control_checksum)) return;
    if(pdr->second_element_is(get_spindle_status_checksum)) {
		// ok this is targeted at us, so set the requ3sted data in the pointer passed into us
		struct spindle_status *t= static_cast<spindle_status*>(pdr->get_data_ptr());
		t->state = this->spindle_on;
		t->current_rpm = this->current_rpm;
		t->target_rpm = this->target_rpm;
		t->current_pwm_value = this->current_pwm_value;
		t->factor= this->factor;
		pdr->set_taken();
    }
}

void PWMSpindleControl::on_set_public_data(void* argument)
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


// returns spindle status
bool PWMSpindleControl::get_alarm(void)
{
	uint32_t debounce = 0;
	while (this->alarm_pin.get()) {
		if ( ++debounce >= 10 ) {
			// pin triggered
			return true;
		}
	}
	return false;
}

// get stall status
bool PWMSpindleControl::get_stall(void)
{
	if (this->spindle_on && this->target_rpm > stall_count_rpm && this->current_rpm < stall_alarm_rpm) {
		if (stall_timer == 0) {
			stall_timer = us_ticker_read();
		} else if (us_ticker_read() - stall_timer > (uint32_t)stall_s * 1000000) {
			return true;
		}
	} else {
		stall_timer = 0;
	}
	return false;
}

void PWMSpindleControl::on_idle(void *argument)
{
	if(THEKERNEL->is_halted()) return;
	// check spindle alarm
    if (this->get_alarm()) {
		THEKERNEL->streams->printf("ALARM: Spindle alarm triggered -  power off/on required\n");
		THEKERNEL->set_halt_reason(SPINDLE_ALARM);
		THEKERNEL->call_event(ON_HALT, nullptr);
		return;
    }
    // check spindle stall
    /*
    if (this->get_stall()) {
		THEKERNEL->streams->printf("ALARM: Spindle stall triggered -  reset required\n");
		THEKERNEL->set_halt_reason(SPINDLE_STALL);
		THEKERNEL->call_event(ON_HALT, nullptr);
    }*/

}

