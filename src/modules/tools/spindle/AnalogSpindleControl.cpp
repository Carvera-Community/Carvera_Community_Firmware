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
#include "Gcode.h"
#include "StreamOutput.h"
#include "modules/tools/accessories/SpindleAccessories.h"
#include "utils.h"

#include <cmath>
#include <new>

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
#define spindle_pwm_deadzone_bottom_checksum CHECKSUM("pwm_deadzone_bottom")
#define spindle_pwm_deadzone_top_checksum    CHECKSUM("pwm_deadzone_top")
#define spindle_pwm_offset_checksum          CHECKSUM("pwm_offset")
#define spindle_pwm_scale_checksum           CHECKSUM("pwm_scale")

#define UPDATE_FREQ 100

namespace {

constexpr float kMinPwmScale = 0.01f;
constexpr float kMaxPwmScale = 5.0f;
constexpr float kMaxDeadzone = 0.95f;
constexpr int kMaxTuneSamples = 51;

struct PwmRpmSample {
    float pwm;
    float rpm;
};

float clamp_float(float value, float lo, float hi, float fallback, bool &changed)
{
    if (!std::isfinite(value)) {
        changed = true;
        return fallback;
    }
    if (value < lo) {
        changed = true;
        return lo;
    }
    if (value > hi) {
        changed = true;
        return hi;
    }
    return value;
}

bool pwm_map_ok(int min_rpm, int max_rpm, float deadzone_bottom, float deadzone_top, float offset, float scale)
{
    if (min_rpm < 0 || max_rpm <= min_rpm)
        return false;
    if (!std::isfinite(deadzone_bottom) || !std::isfinite(deadzone_top) || !std::isfinite(offset) || !std::isfinite(scale))
        return false;
    if (deadzone_bottom < 0.0f || deadzone_top < 0.0f || deadzone_bottom > kMaxDeadzone || deadzone_top > kMaxDeadzone)
        return false;
    if (deadzone_bottom + deadzone_top > kMaxDeadzone)
        return false;
    if (offset < -1.0f || offset > 1.0f)
        return false;
    if (scale < kMinPwmScale || scale > kMaxPwmScale)
        return false;
    return true;
}

bool fit_line(const PwmRpmSample *samples, int begin, int end, float &slope, float &intercept)
{
    const int n = end - begin;
    if (n < 2)
        return false;
    double sx = 0;
    double sy = 0;
    double sxx = 0;
    double sxy = 0;
    for (int i = begin; i < end; i++) {
        const double x = samples[i].pwm;
        const double y = samples[i].rpm;
        sx += x;
        sy += y;
        sxx += x * x;
        sxy += x * y;
    }
    const double dn = n;
    const double denom = dn * sxx - sx * sx;
    if (denom < 1e-9)
        return false;
    slope = static_cast<float>((dn * sxy - sx * sy) / denom);
    intercept = static_cast<float>((sy - static_cast<double>(slope) * sx) / dn);
    return true;
}

}

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

    tuning = false;
    tune_cancel = false;
    read_pwm_map();

    pulses_per_rev = THEKERNEL->config->value(spindle_checksum, spindle_pulses_per_rev_checksum)->as_number(1.0f);
    acc_ratio = THEKERNEL->config->value(spindle_checksum, spindle_acc_ratio_checksum)->as_number(1.0f);
    alarm_pin.from_string(THEKERNEL->config->value(spindle_checksum, spindle_alarm_pin_checksum)->as_string("nc"))->as_input();

    float smoothing_time = THEKERNEL->config->value(spindle_checksum, spindle_control_smoothing_checksum)->as_number(0.1f);
    if (smoothing_time * UPDATE_FREQ < 1.0f)
        smoothing_decay = 1.0f;
    else
        smoothing_decay = 1.0f / (UPDATE_FREQ * smoothing_time);

    // Get the pin for hardware pwm
    {
        Pin *smoothie_pin = new Pin();
        smoothie_pin->from_string(THEKERNEL->config->value(spindle_checksum, spindle_pwm_pin_checksum)->as_string("nc"));
        pwm_pin = smoothie_pin->as_output()->hardware_pwm();
        output_inverted = smoothie_pin->is_inverting();
        delete smoothie_pin;
    }
    // If we got no hardware PWM pin, delete this module
    if (pwm_pin == NULL)
    {
        THEKERNEL->streams->printf("Error: Spindle PWM pin must be P2.0-2.5 or other PWM pin\n");
        delete this;
        return;
    }

    // set pwm frequency
    int period = THEKERNEL->config->value(spindle_checksum, spindle_pwm_period_checksum)->as_int(1000);
    THEKERNEL->Spindle_period_us = period;
    pwm_pin->period_us(period);

    // invert pwm signal if necessary
    pwm_pin->write(output_inverted ? 1 : 0);


    // Get digital out pin for switching the spindle on and off
    {
        Pin *smoothie_pin = new Pin();
        smoothie_pin->from_string(THEKERNEL->config->value(spindle_checksum, spindle_feedback_pin_checksum)->as_string("nc"));
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

    std::string switch_on_pin = THEKERNEL->config->value(spindle_checksum, spindle_switch_on_pin_checksum)->as_string("nc");
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
            current_rpm = rpm_from_pwm(current_pwm_value);
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
        THEKERNEL->streams->printf("ERROR: Spindle alarm triggered -  power off/on required\n");
        THEKERNEL->set_halt_reason(SPINDLE_ALARM);
        THEKERNEL->call_event(ON_HALT, nullptr);
    }
}

void AnalogSpindleControl::apply_pwm_from_targets()
{
    if (tuning)
        return;
    if (!spindle_on || target_rpm <= 0) {
        update_pwm(0);
        return;
    }
    float cmd = target_rpm * (factor / 100.0f);
    if (cmd > (float)max_rpm) cmd = (float)max_rpm;
    update_pwm(pwm_for_rpm(cmd));
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
    if (tuning)
        tune_cancel = true;
    if(switch_on != NULL)
        switch_on->set(false);
    spindle_on = false;
    THEKERNEL->spindleon = false;
    update_pwm(0); // set the PWM value to 0 to make sure it stops
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
    // set the requested PWM value, invert it if necessary
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

void AnalogSpindleControl::read_pwm_map()
{
    min_rpm = THEKERNEL->config->value(spindle_checksum, spindle_min_rpm_checksum)->as_int(100);
    max_rpm = THEKERNEL->config->value(spindle_checksum, spindle_max_rpm_checksum)->as_int(5000);
    pwm_deadzone_bottom = THEKERNEL->config->value(spindle_checksum, spindle_pwm_deadzone_bottom_checksum)->as_number(0.0f);
    pwm_deadzone_top = THEKERNEL->config->value(spindle_checksum, spindle_pwm_deadzone_top_checksum)->as_number(0.0f);
    pwm_offset = THEKERNEL->config->value(spindle_checksum, spindle_pwm_offset_checksum)->as_number(0.0f);
    pwm_scale = THEKERNEL->config->value(spindle_checksum, spindle_pwm_scale_checksum)->as_number(1.0f);
    sanitize_pwm_map();
}

void AnalogSpindleControl::sanitize_pwm_map()
{
    bool changed = false;
    if (min_rpm < 0) {
        min_rpm = 0;
        changed = true;
    }
    if (max_rpm < 1) {
        max_rpm = 1;
        changed = true;
    }
    if (min_rpm >= max_rpm) {
        min_rpm = 0;
        changed = true;
    }
    pwm_deadzone_bottom = clamp_float(pwm_deadzone_bottom, 0.0f, kMaxDeadzone, 0.0f, changed);
    pwm_deadzone_top = clamp_float(pwm_deadzone_top, 0.0f, kMaxDeadzone, 0.0f, changed);
    if (pwm_deadzone_bottom + pwm_deadzone_top > kMaxDeadzone) {
        pwm_deadzone_top = kMaxDeadzone - pwm_deadzone_bottom;
        changed = true;
    }
    pwm_offset = clamp_float(pwm_offset, -1.0f, 1.0f, 0.0f, changed);
    pwm_scale = clamp_float(pwm_scale, kMinPwmScale, kMaxPwmScale, 1.0f, changed);
    if (changed)
        THEKERNEL->streams->printf("ERROR: Analog spindle PWM map config was out of range and was corrected\n");
}

void AnalogSpindleControl::report_map(StreamOutput *stream) const
{
    stream->printf("min_rpm: %d max_rpm: %d deadzone_bottom: %1.4f deadzone_top: %1.4f offset: %1.4f scale: %1.4f\n",
                   min_rpm, max_rpm, pwm_deadzone_bottom, pwm_deadzone_top, pwm_offset, pwm_scale);
}

float AnalogSpindleControl::pwm_for_rpm(float rpm) const
{
    if (rpm <= 0.0f || max_rpm <= 0)
        return 0.0f;
    float duty = pwm_offset + pwm_scale * (rpm / static_cast<float>(max_rpm));
    float lo = pwm_deadzone_bottom;
    float hi = 1.0f - pwm_deadzone_top;
    if (lo < 0.0f)
        lo = 0.0f;
    if (hi > 1.0f)
        hi = 1.0f;
    if (hi < lo)
        hi = lo;
    if (duty < lo)
        duty = lo;
    if (duty > hi)
        duty = hi;
    return duty;
}

float AnalogSpindleControl::rpm_from_pwm(float duty) const
{
    if (duty <= 0.0f || pwm_scale <= 0.0f || max_rpm <= 0)
        return 0.0f;
    float rpm = (duty - pwm_offset) * static_cast<float>(max_rpm) / pwm_scale;
    if (rpm < 0.0f)
        rpm = 0.0f;
    if (rpm > static_cast<float>(max_rpm))
        rpm = static_cast<float>(max_rpm);
    return rpm;
}

bool AnalogSpindleControl::wait_for_stable_rpm(float &rpm, uint32_t min_ms, uint32_t max_ms)
{
    float window[4];
    int count = 0;
    int idx = 0;
    uint32_t elapsed = 0;
    while (elapsed < max_ms) {
        safe_delay_ms(200);
        elapsed += 200;
        if (tune_cancel || THEKERNEL->is_halted())
            return false;
        window[idx] = current_rpm;
        idx = (idx + 1) % 4;
        if (count < 4)
            count++;
        if (elapsed < min_ms || count < 4)
            continue;
        float lo = window[0];
        float hi = window[0];
        float sum = 0.0f;
        for (int i = 0; i < 4; i++) {
            if (window[i] < lo)
                lo = window[i];
            if (window[i] > hi)
                hi = window[i];
            sum += window[i];
        }
        const float mean = sum / 4.0f;
        float tol = mean * 0.04f;
        if (tol < 80.0f)
            tol = 80.0f;
        if (hi - lo <= tol) {
            rpm = mean;
            return true;
        }
    }
    rpm = current_rpm;
    return !tune_cancel && !THEKERNEL->is_halted();
}

namespace {

// Find the rising region between the stopped (bottom dead zone) and plateau
// (top dead zone) portions of the sweep, fit a line through it, and derive the
// PWM map parameters from the inverse of that line.
bool fit_pwm_samples(const PwmRpmSample *samples, int count, StreamOutput *stream,
                     int &min_rpm, int &max_rpm, float &deadzone_bottom, float &deadzone_top,
                     float &offset, float &scale, float &fit_error)
{
    float peak = 0.0f;
    for (int i = 0; i < count; i++) {
        if (samples[i].rpm > peak)
            peak = samples[i].rpm;
    }
    if (peak < 100.0f) {
        stream->printf("ERROR: Analog spindle auto-tune failed, no RPM feedback\n");
        return false;
    }

    float stopped = peak * 0.02f;
    if (stopped < 80.0f)
        stopped = 80.0f;
    int spin = -1;
    for (int i = 0; i < count; i++) {
        if (samples[i].rpm >= stopped) {
            spin = i;
            break;
        }
    }
    if (spin < 0) {
        stream->printf("ERROR: Analog spindle auto-tune failed, no RPM feedback\n");
        return false;
    }

    float tol = peak * 0.03f;
    if (tol < 120.0f)
        tol = 120.0f;
    int plateau = count - 1;
    if (samples[count - 1].rpm >= peak - tol) {
        for (int i = count - 1; i >= spin; i--) {
            if (samples[i].rpm >= peak - tol)
                plateau = i;
            else
                break;
        }
    }
    if (plateau <= spin) {
        stream->printf("ERROR: Analog spindle auto-tune failed, RPM did not increase with PWM\n");
        return false;
    }

    float slope = 0.0f;
    float intercept = 0.0f;
    if (!fit_line(samples, spin, plateau + 1, slope, intercept) || slope <= 1.0f) {
        stream->printf("ERROR: Analog spindle auto-tune failed, RPM did not increase with PWM\n");
        return false;
    }

    // A stopped sample that the line still predicts as stopped is the low end of
    // the line, not a dead zone. A dead zone is a sample the line says should
    // already be turning.
    deadzone_bottom = samples[spin].pwm;
    if (spin == 0 || (slope * samples[spin - 1].pwm + intercept) < stopped)
        deadzone_bottom = 0.0f;
    deadzone_top = 1.0f - samples[plateau].pwm;
    double plateau_sum = 0.0;
    int plateau_n = 0;
    for (int i = plateau; i < count; i++) {
        plateau_sum += samples[i].rpm;
        plateau_n++;
    }
    max_rpm = static_cast<int>(std::lround(plateau_sum / plateau_n));
    min_rpm = static_cast<int>(std::lround(samples[spin].rpm));
    if (min_rpm < 1)
        min_rpm = 1;
    offset = -intercept / slope;
    scale = static_cast<float>(max_rpm) / slope;

    fit_error = 0.0f;
    for (int i = spin; i <= plateau; i++) {
        const float predicted = slope * samples[i].pwm + intercept;
        const float err = std::fabs(predicted - samples[i].rpm);
        if (err > fit_error)
            fit_error = err;
    }

    if (!pwm_map_ok(min_rpm, max_rpm, deadzone_bottom, deadzone_top, offset, scale)) {
        stream->printf("ERROR: Analog spindle auto-tune failed, fitted map is out of range\n");
        stream->printf("min_rpm: %d max_rpm: %d deadzone_bottom: %1.4f deadzone_top: %1.4f offset: %1.4f scale: %1.4f\n",
                       min_rpm, max_rpm, deadzone_bottom, deadzone_top, offset, scale);
        return false;
    }
    return true;
}

}

void AnalogSpindleControl::auto_tune(StreamOutput *stream, float step)
{
    if (tuning)
        return;
    if (THEKERNEL->is_halted()) {
        stream->printf("ERROR: Analog spindle auto-tune ignored while halted\n");
        return;
    }
    if (THEKERNEL->get_laser_mode()) {
        stream->printf("ERROR: Analog spindle auto-tune is not available in laser mode\n");
        return;
    }
    if (feedback_pin == NULL) {
        stream->printf("ERROR: Analog spindle auto-tune requires spindle.feedback_pin\n");
        return;
    }
    if (!(step > 0.0f))
        step = 0.05f;
    if (step < 0.02f)
        step = 0.02f;
    if (step > 0.20f)
        step = 0.20f;
    int intervals = static_cast<int>(1.0f / step + 0.5f);
    if (intervals < 2)
        intervals = 2;
    if (intervals > kMaxTuneSamples - 1)
        intervals = kMaxTuneSamples - 1;
    const int count = intervals + 1;
    step = 1.0f / static_cast<float>(intervals);

    PwmRpmSample *samples = new (std::nothrow) PwmRpmSample[count]();
    if (samples == nullptr) {
        stream->printf("ERROR: Analog spindle auto-tune failed, not enough memory\n");
        return;
    }

    stream->printf("Analog spindle auto-tune, PWM step %1.3f. The spindle will run.\n", step);
    tune_cancel = false;
    tuning = true;
    turn_on();
    update_pwm(0.0f);
    if (THEKERNEL->spindle_accessories != nullptr)
        THEKERNEL->spindle_accessories->spindle_started();

    bool ok = true;
    for (int i = 0; i < count; i++) {
        if (tune_cancel || THEKERNEL->is_halted()) {
            ok = false;
            break;
        }
        float pwm = step * static_cast<float>(i);
        if (i == count - 1)
            pwm = 1.0f;
        update_pwm(pwm);
        float rpm = 0.0f;
        const uint32_t max_ms = (i == 0) ? 8000 : 5000;
        if (!wait_for_stable_rpm(rpm, 1000, max_ms)) {
            ok = false;
            break;
        }
        samples[i].pwm = pwm;
        samples[i].rpm = rpm;
        stream->printf("pwm %1.3f rpm %5.0f\n", pwm, rpm);
    }
    if (ok && (tune_cancel || THEKERNEL->is_halted()))
        ok = false;

    int new_min = 0;
    int new_max = 0;
    float new_bottom = 0.0f;
    float new_top = 0.0f;
    float new_offset = 0.0f;
    float new_scale = 1.0f;
    float fit_error = 0.0f;
    if (ok) {
        ok = fit_pwm_samples(samples, count, stream, new_min, new_max, new_bottom, new_top, new_offset, new_scale, fit_error);
    } else {
        stream->printf("ERROR: Analog spindle auto-tune aborted\n");
    }
    delete[] samples;

    tuning = false;
    if (ok) {
        min_rpm = new_min;
        max_rpm = new_max;
        pwm_deadzone_bottom = new_bottom;
        pwm_deadzone_top = new_top;
        pwm_offset = new_offset;
        pwm_scale = new_scale;
        stream->printf("Auto-tune fit error %5.0f rpm\n", fit_error);
    }
    bool needs_stop = spindle_on;
    if (needs_stop)
        turn_off();
    if (needs_stop && THEKERNEL->spindle_accessories != nullptr)
        THEKERNEL->spindle_accessories->spindle_stopped();
    if (ok) {
        stream->printf("These values are temporary\nand will need to be saved to the config file with\n");
        stream->printf("config-set sd spindle.min_rpm %d\n", min_rpm);
        stream->printf("config-set sd spindle.max_rpm %d\n", max_rpm);
        stream->printf("config-set sd spindle.pwm_deadzone_bottom %1.4f\n", pwm_deadzone_bottom);
        stream->printf("config-set sd spindle.pwm_deadzone_top %1.4f\n", pwm_deadzone_top);
        stream->printf("config-set sd spindle.pwm_offset %1.4f\n", pwm_offset);
        stream->printf("config-set sd spindle.pwm_scale %1.4f\n", pwm_scale);
    }
}

void AnalogSpindleControl::on_analog_settings(Gcode *gcode)
{
    if (tuning)
        return;
    // M959.1 sweeps PWM and fits min/max RPM, dead zones, offset, and scale from feedback.
    if (gcode->subcode == 1) {
        float step = 0.05f;
        if (gcode->has_letter('P'))
            step = gcode->get_value('P');
        auto_tune(gcode->stream, step);
        return;
    }
    if (gcode->subcode != 0)
        return;

    const bool any = gcode->has_letter('L') || gcode->has_letter('H') || gcode->has_letter('B') ||
                     gcode->has_letter('T') || gcode->has_letter('O') || gcode->has_letter('S');
    if (!any) {
        report_map(gcode->stream);
        return;
    }

    const int new_min = gcode->has_letter('L') ? gcode->get_int('L') : min_rpm;
    const int new_max = gcode->has_letter('H') ? gcode->get_int('H') : max_rpm;
    const float new_bottom = gcode->has_letter('B') ? gcode->get_value('B') : pwm_deadzone_bottom;
    const float new_top = gcode->has_letter('T') ? gcode->get_value('T') : pwm_deadzone_top;
    const float new_offset = gcode->has_letter('O') ? gcode->get_value('O') : pwm_offset;
    const float new_scale = gcode->has_letter('S') ? gcode->get_value('S') : pwm_scale;
    if (!pwm_map_ok(new_min, new_max, new_bottom, new_top, new_offset, new_scale)) {
        gcode->stream->printf("ERROR: Analog spindle PWM map values are out of range\n");
        return;
    }
    min_rpm = new_min;
    max_rpm = new_max;
    pwm_deadzone_bottom = new_bottom;
    pwm_deadzone_top = new_top;
    pwm_offset = new_offset;
    pwm_scale = new_scale;
    apply_pwm_from_targets();
    report_map(gcode->stream);
}
