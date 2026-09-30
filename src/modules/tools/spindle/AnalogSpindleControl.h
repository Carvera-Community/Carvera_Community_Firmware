/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ANALOG_SPINDLE_MODULE_H
#define ANALOG_SPINDLE_MODULE_H

#include "SpindleControl.h"
#include <stdint.h>
#include "Pin.h"

namespace mbed {
    class PwmOut;
    class InterruptIn;
}

class Gcode;
class StreamOutput;

// Open-loop PWM spindle. While running, duty = offset + scale * (rpm / max_rpm),
// then limited to [deadzone_bottom, 1 - deadzone_top]. Off writes duty 0.
class AnalogSpindleControl: public SpindleControl {
    public:
        AnalogSpindleControl() : feedback_pin(nullptr) {};
        virtual ~AnalogSpindleControl() {};
        void on_module_loaded();
        void on_get_public_data(void* argument);
        void on_set_public_data(void* argument);
        void on_idle(void* argument);

    private:
        void on_pin_rise();
        uint32_t on_update_speed(uint32_t dummy);
        bool get_alarm(void);

        void apply_pwm_from_targets(void);

        Pin *switch_on;
        mbed::PwmOut *pwm_pin;
        mbed::InterruptIn *feedback_pin;
        bool output_inverted;

        float target_rpm;
        float current_rpm;
        float current_pwm_value;
        float factor;
        int min_rpm;
        int max_rpm;
        // Duty added to the scaled command, and PWM change across 0..max_rpm.
        float pwm_offset;
        float pwm_scale;
        // Lowest duty that turns the spindle, and unused duty at the top of the range.
        float pwm_deadzone_bottom;
        float pwm_deadzone_top;
        int delay_s;
        int delay_on_s;
        int delay_off_s;
        bool tuning;
        bool tune_cancel;

        float pulses_per_rev;
        float acc_ratio;
        float smoothing_decay;
        int time_since_update;
        uint32_t last_rev_time;
        volatile uint32_t rev_time;
        volatile uint32_t irq_count;
        Pin alarm_pin;

        void turn_on(void);
        void turn_off(void);
        void set_speed(int);
        void report_speed(void);
        void update_pwm(float);
        void set_factor(float);
        void on_analog_settings(Gcode *gcode) override;

        void read_pwm_map(void);
        void sanitize_pwm_map(void);
        void report_map(StreamOutput *stream) const;
        float pwm_for_rpm(float rpm) const;
        float rpm_from_pwm(float duty) const;
        void auto_tune(StreamOutput *stream, float step, uint32_t step_ms, int sweeps, bool apply, bool validate);
        void validate_map(StreamOutput *stream, float step);
        bool sample_commanded_speeds(StreamOutput *stream, int intervals, uint32_t min_ms, uint32_t max_ms, float &fluctuation);
        bool measure_start_stop_times(StreamOutput *stream, int &on_s, int &off_s);
        void dwell_seconds(int seconds);
        bool wait_for_stable_rpm(float &rpm, uint32_t min_ms, uint32_t max_ms);
};

#endif
