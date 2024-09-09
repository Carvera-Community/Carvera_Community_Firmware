/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "Gcode.h"
#include "Conveyor.h"
#include "SpindleControl.h"
#include "libs/StreamOutputPool.h"
#include "libs/PublicData.h"
#include "SwitchPublicAccess.h"
#include "ATCHandlerPublicAccess.h"

void SpindleControl::on_gcode_received(void *argument) 
{
    
    Gcode *gcode = static_cast<Gcode *>(argument);
        
    if (gcode->has_m)
    {
        if (gcode->m == 957)
        {
            // M957: report spindle speed
            report_speed();
        }
        else if (gcode->m == 958)
        {
            THECONVEYOR->wait_for_idle();
            // M958: set spindle PID parameters
            if (gcode->has_letter('P'))
                set_p_term( gcode->get_value('P') );
            if (gcode->has_letter('I'))
                set_i_term( gcode->get_value('I') );
            if (gcode->has_letter('D'))
                set_d_term( gcode->get_value('D') );
            // report PID settings
            report_settings();
          
        }
        else if (gcode->m == 3)
        {
        	if (!THEKERNEL->get_laser_mode()) {
                // current tool number and tool offset
                struct tool_status tool;
                bool tool_ok = PublicData::get_value( atc_handler_checksum, get_tool_status_checksum, &tool );
                if (tool_ok) {
                	tool_ok = tool.active_tool > 0;
                }
            	// check if is tool -1 or tool 0
            	if (!tool_ok) {
        			THEKERNEL->call_event(ON_HALT, nullptr);
        			THEKERNEL->set_halt_reason(MANUAL);
        			THEKERNEL->streams->printf("ERROR: No tool or probe tool!\n");
        			return;
            	}

                THECONVEYOR->wait_for_idle();
                // open vacuum if set
            	if (THEKERNEL->get_vacuum_mode()) {
            		// open vacuum
            		bool b = true;
                    PublicData::set_value( switch_checksum, vacuum_checksum, state_checksum, &b );
            	}

                // M3 with S value provided: set speed
                if (gcode->has_letter('S'))
                {
                    set_speed(gcode->get_value('S'));
                }
                // M3: Spindle on
                if (!spindle_on) {
                    turn_on();
                }
        	}
        }
        else if (gcode->m == 5)
        {
        	if (!THEKERNEL->get_laser_mode()) {
                THECONVEYOR->wait_for_idle();

                // close vacuum if set
            	if (THEKERNEL->get_vacuum_mode()) {
            		// close vacuum
            		bool b = false;
                    PublicData::set_value( switch_checksum, vacuum_checksum, state_checksum, &b );
            	}

                // M5: spindle off
                if (spindle_on) {
                    turn_off();
                }
        	}
        }
        else if (gcode->m == 223)
        {	// M222 - rpm override percentage
            if (gcode->has_letter('S')) {
                float factor = gcode->get_value('S');
                // enforce minimum 50% speed
                if (factor < 50.0F)
                    factor = 50.0F;
                // enforce maximum 2x speed
                if (factor > 200.0F)
                    factor = 200.0F;
                set_factor(factor);
            }
        }
    }

}

void SpindleControl::on_halt(void *argument)
{
    if (argument == nullptr) {
        if(spindle_on) {
            turn_off();
        }
    }
}
