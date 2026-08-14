/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "SpindleControl.h"

#include "ATCHandlerPublicAccess.h"
#include "Conveyor.h"
#include "Gcode.h"
#include "libs/Kernel.h"
#include "libs/PublicData.h"
#include "libs/StreamOutputPool.h"
#include "modules/tools/accessories/SpindleAccessories.h"

void SpindleControl::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);
    if (!gcode->has_m)
        return;

    if (gcode->m == 957) {
        report_speed();
        return;
    }
    if (gcode->m == 958) {
        THECONVEYOR->wait_for_idle();
        if (gcode->has_letter('P'))
            set_p_term(gcode->get_value('P'));
        if (gcode->has_letter('I'))
            set_i_term(gcode->get_value('I'));
        if (gcode->has_letter('D'))
            set_d_term(gcode->get_value('D'));
        report_settings();
        return;
    }
    if (gcode->m == 223) {
        if (gcode->has_letter('S')) {
            float factor = gcode->get_value('S');
            if (factor < 10.0F)
                factor = 10.0F;
            if (factor > 300.0F)
                factor = 300.0F;
            set_factor(factor);
        }
        return;
    }
    if (gcode->m != 3 && gcode->m != 5)
        return;
    if (handling_gcode || (gcode->m == 3 && THEKERNEL->is_halted()))
        return;

    if (gcode->m == 3) {
        if (!THEKERNEL->get_laser_mode()) {
            tool_status tool{};
            const bool tool_ok = PublicData::get_value(atc_handler_checksum, get_tool_status_checksum, &tool) &&
                                 tool.active_tool > 0 && tool.active_tool < 100000;
            if (!tool_ok) {
                THEKERNEL->set_halt_reason(MANUAL);
                THEKERNEL->call_event(ON_HALT, nullptr);
                THEKERNEL->streams->printf("ERROR: Spindle cannot run without a valid tool\n");
                return;
            }
        }

        handling_gcode = true;
        if (!THEKERNEL->get_laser_mode()) {
            THECONVEYOR->wait_for_idle();
            if (gcode->has_letter('S'))
                set_speed(gcode->get_value('S'));
            if (!spindle_on)
                turn_on();
        }
        THEKERNEL->spindle_accessories->spindle_started();
        handling_gcode = false;
        return;
    }

    handling_gcode = true;
    if (!THEKERNEL->get_laser_mode()) {
        THECONVEYOR->wait_for_idle();
        if (spindle_on)
            turn_off();
    }
    THEKERNEL->spindle_accessories->spindle_stopped();
    handling_gcode = false;
}

void SpindleControl::on_halt(void *argument)
{
    if (argument != nullptr)
        return;
    if (spindle_on)
        turn_off();
    THEKERNEL->spindle_accessories->spindle_stopped();
}
