/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef PID_PWM_SPINDLE_MODULE_H
#define PID_PWM_SPINDLE_MODULE_H

#include "PWMSpindleControl.h"
#include <stdint.h>
#include "Pin.h"
#include <vector>

namespace mbed {
    class PwmOut;
    class InterruptIn;
}

class PIDPWMSpindleControl: public PWMSpindleControl {
    public:
        PIDPWMSpindleControl();
        virtual ~PIDPWMSpindleControl() {};
        void on_module_loaded();

    private:
        void on_pin_rise();
        uint32_t on_update_speed(uint32_t dummy);
        
        float ff_slope;
        float ff_offset;

        std::vector<uint32_t> pulse_times;
        uint32_t pulse_idx{0};
        uint32_t total_pulse_time{0};
};

#endif
