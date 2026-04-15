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

// Analog 0–10V spindle control with optional tachometer feedback and alarm input (same config keys as PWM spindle).
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
};

#endif
