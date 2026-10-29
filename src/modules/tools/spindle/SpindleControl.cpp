/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "SpindleControl.h"

#include "ATCHandlerPublicAccess.h"
#include "Config.h"
#include "ConfigValue.h"
#include "Conveyor.h"
#include "Gcode.h"
#include "checksumm.h"
#include "libs/Kernel.h"
#include "libs/PublicData.h"
#include "libs/StreamOutputPool.h"
#include "modules/tools/accessories/SpindleAccessories.h"

#define spindle_checksum CHECKSUM("spindle")
#define three_d_toolsetter_checksum CHECKSUM("3dtoolsetter")

static void send_internal_command(const char *cmd)
{
    string g(cmd);
    Gcode gc(g, &(StreamOutput::NullStream));
    THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc);
}

void SpindleControl::load_3dtoolsetter_config()
{
    this->enable_3dtoolsetter = THEKERNEL->config->value(spindle_checksum, three_d_toolsetter_checksum)->as_bool(false);
}

void SpindleControl::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);
    if (!gcode->has_m) {
        return;
    }

    if (gcode->m == 957) {
        report_speed();
        return;
    }

    if (gcode->m == 958) {
        THECONVEYOR->wait_for_idle();
        if (gcode->has_letter('P')) {
            set_p_term(gcode->get_value('P'));
        }
        if (gcode->has_letter('I')) {
            set_i_term(gcode->get_value('I'));
        }
        if (gcode->has_letter('D')) {
            set_d_term(gcode->get_value('D'));
        }
        report_settings();
        return;
    }

    if (gcode->m == 959) {
        on_analog_settings(gcode);
        return;
    }

    if (gcode->m == 223) {
        if (gcode->has_letter('S')) {
            float factor = gcode->get_value('S');
            if (factor < 10.0F) {
                factor = 10.0F;
            }
            if (factor > 300.0F) {
                factor = 300.0F;
            }
            set_factor(factor);
        }
        return;
    }

    if (gcode->m == 4 && this->supports_3dtoolsetter_m4 && !this->enable_3dtoolsetter) {
        THEKERNEL->streams->printf("ERROR: M4 is disabled unless spindle.3dtoolsetter is true\n");
        return;
    }

    if (gcode->m != 3 && gcode->m != 4 && gcode->m != 5) {
        return;
    }

    if (handling_gcode || ((gcode->m == 3 || gcode->m == 4) && THEKERNEL->is_halted())) {
        return;
    }

    if (gcode->m == 3 || gcode->m == 4) {
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

        // In 3dtoolsetter mode, M4 maps to EXT direction on before spindle start.
        if (this->supports_3dtoolsetter_m4 && this->enable_3dtoolsetter && gcode->m == 4) {
            send_internal_command("M851 S100");
        }

        if (!THEKERNEL->get_laser_mode()) {
            THECONVEYOR->wait_for_idle();
            if (gcode->has_letter('S')) {
                set_speed(gcode->get_value('S'));
            }
            if (!spindle_on) {
                turn_on();
            }
        }

        THEKERNEL->spindle_accessories->spindle_started();
        handling_gcode = false;
        return;
    }

    handling_gcode = true;

    if (!THEKERNEL->get_laser_mode()) {
        THECONVEYOR->wait_for_idle();
        if (spindle_on) {
            turn_off();
        }
    }

    if (this->supports_3dtoolsetter_m4 && this->enable_3dtoolsetter) {
        send_internal_command("M852");
    }

    THEKERNEL->spindle_accessories->spindle_stopped();
    handling_gcode = false;
}

void SpindleControl::on_analog_settings(Gcode *gcode)
{
    gcode->stream->printf("ERROR: M959 requires an analog spindle type\n");
}

void SpindleControl::on_halt(void *argument)
{
    if (argument != nullptr) {
        return;
    }

    if (spindle_on) {
        turn_off();
    }

    if (this->supports_3dtoolsetter_m4 && this->enable_3dtoolsetter) {
        send_internal_command("M852");
    }

    THEKERNEL->spindle_accessories->spindle_stopped();
}
