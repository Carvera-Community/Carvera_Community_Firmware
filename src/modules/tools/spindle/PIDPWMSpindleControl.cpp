/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "PIDPWMSpindleControl.h"
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

#define spindle_checksum                    CHECKSUM("spindle")
#define spindle_ff_slope_checksum          CHECKSUM("ff_slope")
#define spindle_ff_offset_checksum          CHECKSUM("ff_offset")

#define UPDATE_FREQ 100

#include "libs/Pin.h"
#include "Gcode.h"
#include "InterruptIn.h"
#include "PwmOut.h"
#include "port_api.h"
#include "us_ticker_api.h"

#include <numeric>

PIDPWMSpindleControl::PIDPWMSpindleControl()
{
}

void PIDPWMSpindleControl::on_module_loaded()
{
    PWMSpindleControl::on_module_loaded();
    
    ff_slope = THEKERNEL->config->value(spindle_checksum, spindle_ff_slope_checksum)->by_default(0.0000485f)->as_number();
    ff_offset = THEKERNEL->config->value(spindle_checksum, spindle_ff_offset_checksum)->by_default(0.02f)->as_number();
    
    pulse_times = std::vector<uint32_t>(static_cast<uint32_t>(std::ceil(pulses_per_rev)), 0xFFFFFF);
    total_pulse_time = std::accumulate(pulse_times.begin(), pulse_times.end(), 0);
    
    // Re-attach interrupt and ticker to the PID overrides.
    if (feedback_pin != NULL) {
        feedback_pin->rise(this, &PIDPWMSpindleControl::on_pin_rise);
    }
    THEKERNEL->slow_ticker->attach(UPDATE_FREQ, this, &PIDPWMSpindleControl::on_update_speed);
}

void PIDPWMSpindleControl::on_pin_rise() {
    uint32_t timestamp = us_ticker_read();
    uint32_t time_diff = timestamp - last_edge;
    total_pulse_time -= pulse_times[pulse_idx];
    total_pulse_time += time_diff;
    pulse_times[pulse_idx] = time_diff;
    last_edge = timestamp;
    pulse_idx = (pulse_idx + 1) % pulse_times.size();
    time_since_update = 0;
}

uint32_t PIDPWMSpindleControl::on_update_speed(uint32_t /*dummy*/) {
    if (++time_since_update > UPDATE_FREQ) {
    	current_rpm = 0;
    } else {
	    uint32_t t = total_pulse_time;
	    if (t > 2000 * acc_ratio ) {	
	        float new_rpm = 1000000 * acc_ratio * 60.0f / t;
	        current_rpm = smoothing_decay * new_rpm + (1.0f - smoothing_decay) * current_rpm;
	    }
	}

    if (spindle_on) {
        float error = target_rpm * (factor / 100) - current_rpm;

        if (current_pwm_value < 1.0f || error <= 0) {
            current_I_value += control_I_term * error * 1.0f / UPDATE_FREQ;
            current_I_value = confine(current_I_value, -1.0f, 1.0f);
        }

        float new_pwm = ff_slope * target_rpm + ff_offset; 
        new_pwm += control_P_term * error;
        new_pwm += current_I_value;
        new_pwm += control_D_term * (error - prev_error) * UPDATE_FREQ;
        new_pwm = confine(new_pwm, 0.0f, max_pwm);

        prev_error = error;
        current_pwm_value = new_pwm;
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
