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
#include "PlayerPublicAccess.h"
#include "libs/Kernel.h"
#include "libs/PublicData.h"
#include "libs/StreamOutputPool.h"
#include "modules/tools/accessories/SpindleAccessories.h"
#include "us_ticker_api.h"

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
    if (gcode->m == 959) {
        on_analog_settings(gcode);
        return;
    }
    if (gcode->m == 960) {
        on_modbus_parameter(gcode);
        return;
    }
    if (gcode->m == 19) {
        on_spindle_orient(gcode);
        return;
    }
    if (gcode->m == 961) {
        on_spindle_angle(gcode);
        return;
    }
    if (gcode->m == 577) {
        on_link_check(gcode);
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
    if (gcode->m != 3 && gcode->m != 4 && gcode->m != 5)
        return;
    if (gcode->m == 4 && !supports_direction()) {
        halt_for_command("ERROR: M4 requires a spindle that supports direction");
        return;
    }
    if (handling_gcode || ((gcode->m == 3 || gcode->m == 4) && THEKERNEL->is_halted()))
        return;

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
        if (!THEKERNEL->get_laser_mode()) {
            THECONVEYOR->wait_for_idle();
            set_ccw(gcode->m == 4);
            if (gcode->has_letter('S'))
                set_speed(gcode->get_value('S'));
            if (!spindle_on)
                turn_on();
            else
                reassert_run();
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

void SpindleControl::on_analog_settings(Gcode *gcode)
{
    gcode->stream->printf("ERROR: M959 requires an analog spindle type\n");
}

bool SpindleControl::modbus_exchange(bool, uint16_t, uint16_t, uint16_t *)
{
    return false;
}

bool SpindleControl::modbus_read(uint16_t reg, uint16_t words)
{
    if (words != 1)
        return false;
    uint16_t value = 0;
    return modbus_exchange(false, reg, 0, &value);
}

void SpindleControl::on_modbus_parameter(Gcode *gcode)
{
    if (!supports_modbus()) {
        gcode->stream->printf("ERROR: M960 requires a Modbus spindle\n");
        return;
    }
    if (!modbus_ready()) {
        gcode->stream->printf("ERROR: Modbus spindle is not connected\n");
        return;
    }
    if (!gcode->has_letter('P')) {
        gcode->stream->printf("M960 P<register> [S<value>]\n");
        return;
    }
    const float address = gcode->get_value('P');
    if (address < 0.0F || address > 65535.0F) {
        gcode->stream->printf("ERROR: register must be 0 to 65535\n");
        return;
    }
    const auto reg = static_cast<uint16_t>(address);
    if (gcode->has_letter('S')) {
        void *playing = nullptr;
        if (PublicData::get_value(player_checksum, is_playing_checksum, &playing) && playing != nullptr &&
            *static_cast<bool *>(playing)) {
            gcode->stream->printf("ERROR: M960 cannot write registers while a job is playing\n");
            return;
        }
        const float raw = gcode->get_value('S');
        if (raw < -32768.0F || raw > 65535.0F) {
            gcode->stream->printf("ERROR: value must fit in a 16-bit register\n");
            return;
        }
        const auto word = raw < 0.0F ? static_cast<uint16_t>(static_cast<int16_t>(raw))
                                      : static_cast<uint16_t>(raw);
        if (!modbus_exchange(true, reg, word, nullptr)) {
            gcode->stream->printf("ERROR: register %u write failed\n", reg);
            return;
        }
        gcode->stream->printf("M960 P%u S%u\n", reg, word);
        return;
    }
    uint16_t value = 0;
    if (!modbus_exchange(false, reg, 0, &value)) {
        gcode->stream->printf("ERROR: register %u read failed\n", reg);
        return;
    }
    gcode->stream->printf("M960 P%u S%u\n", reg, value);
}

void SpindleControl::on_spindle_orient(Gcode *gcode)
{
    gcode->stream->printf("ERROR: M19 requires a spindle that can orient\n");
}

void SpindleControl::on_spindle_angle(Gcode *gcode)
{
    gcode->stream->printf("ERROR: M961 requires a Tonghang spindle\n");
}

void SpindleControl::halt_for_command(const char *message)
{
    if (THEKERNEL->is_halted())
        return;
    THEKERNEL->set_halt_reason(MANUAL);
    THEKERNEL->call_event(ON_HALT, nullptr);
    THEKERNEL->streams->printf("%s\n", message);
}

void SpindleControl::on_link_check(Gcode *gcode)
{
    if (!supports_modbus()) {
        gcode->stream->printf("ERROR: M577 requires a Modbus spindle\n");
        return;
    }
    if (!modbus_ready()) {
        gcode->stream->printf("ERROR: Modbus spindle is not connected\n");
        return;
    }
    int samples = 20;
    if (gcode->has_letter('P'))
        samples = gcode->get_int('P');
    if (samples < 1 || samples > 50) {
        gcode->stream->printf("ERROR: M577 P must be 1 to 50\n");
        return;
    }

    uint16_t reg = 0;
    uint16_t words = 1;
    if (gcode->has_letter('R')) {
        const float address = gcode->get_value('R');
        if (address < 0.0F || address > 65535.0F) {
            gcode->stream->printf("ERROR: register must be 0 to 65535\n");
            return;
        }
        reg = static_cast<uint16_t>(address);
        words = 1;
    } else if (!modbus_link_defaults(&reg, &words)) {
        gcode->stream->printf("ERROR: M577 requires R\n");
        return;
    }

    if (!modbus_wait()) {
        gcode->stream->printf("ERROR: M577 Modbus link is busy\n");
        return;
    }

    modbus_lock(true);
    uint32_t min_us = 0xFFFFFFFF;
    uint32_t max_us = 0;
    uint32_t sum_us = 0;
    int ok = 0;
    for (int i = 0; i < samples; ++i) {
        THEKERNEL->call_event(ON_IDLE, this);
        const uint32_t t0 = us_ticker_read();
        if (!modbus_read(reg, words))
            break;
        const uint32_t dt = static_cast<uint32_t>(us_ticker_read() - t0);
        if (dt < min_us)
            min_us = dt;
        if (dt > max_us)
            max_us = dt;
        sum_us += dt;
        ++ok;
    }
    modbus_lock(false);

    if (ok == 0) {
        gcode->stream->printf("ERROR: M577 register %u read failed\n", reg);
        return;
    }
    const float avg_ms = static_cast<float>(sum_us) / static_cast<float>(ok) / 1000.0F;
    const uint32_t baud = modbus_baud();
    if (baud != 0) {
        gcode->stream->printf("M577: %d samples, min %.2f ms, avg %.2f ms, max %.2f ms, %lu baud\n", ok,
                              static_cast<double>(min_us) / 1000.0, static_cast<double>(avg_ms),
                              static_cast<double>(max_us) / 1000.0, static_cast<unsigned long>(baud));
        return;
    }
    gcode->stream->printf("M577: %d samples, min %.2f ms, avg %.2f ms, max %.2f ms\n", ok,
                          static_cast<double>(min_us) / 1000.0, static_cast<double>(avg_ms),
                          static_cast<double>(max_us) / 1000.0);
}

void SpindleControl::on_halt(void *argument)
{
    if (argument != nullptr)
        return;
    if (spindle_on)
        turn_off();
    THEKERNEL->spindle_accessories->spindle_stopped();
}
