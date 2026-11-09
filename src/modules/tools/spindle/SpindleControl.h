/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef SPINDLE_CONTROL_MODULE_H
#define SPINDLE_CONTROL_MODULE_H

#include <cstdint>

#include "libs/Module.h"

class Gcode;

class SpindleControl: public Module {
    public:
        SpindleControl() : spindle_on(false), handling_gcode(false) {};
        virtual ~SpindleControl() {};
        virtual void on_module_loaded() {};

    protected:
        bool spindle_on;
        bool handling_gcode;
        void halt_for_command(const char *message);

    private:
        void on_gcode_received(void *argument);
        void on_halt(void *argument);
        
        virtual void turn_on(void) {};
        virtual void turn_off(void) {};
        virtual void set_ccw(bool) {};
        // M3/M4 while the spindle is already on. The hold after an orient is "on"
        // but not turning; this is what starts it again.
        virtual void reassert_run() {};
        virtual bool supports_direction() const { return false; }
        virtual void set_speed(int) {};
        virtual void report_speed(void) {};
        virtual void set_p_term(float) {};
        virtual void set_i_term(float) {};
        virtual void set_d_term(float) {};
        virtual void report_settings(void) {};
        virtual void on_analog_settings(Gcode *);
        virtual bool supports_modbus() const { return false; }
        virtual bool modbus_ready() const { return false; }
        // One holding-register read or write. out receives the register on a read.
        virtual bool modbus_exchange(bool writing, uint16_t reg, uint16_t value, uint16_t *out);
        // M577 when R is omitted. False means the command has to pass R.
        virtual bool modbus_link_defaults(uint16_t *, uint16_t *) const { return false; }
        virtual bool modbus_read(uint16_t reg, uint16_t words);
        virtual bool modbus_wait() { return true; }
        virtual void modbus_lock(bool) {}
        virtual uint32_t modbus_baud() const { return 0; }
        virtual void on_modbus_parameter(Gcode *);
        virtual void on_spindle_orient(Gcode *);
        virtual void on_spindle_angle(Gcode *);
        virtual void on_link_check(Gcode *);

        virtual void set_factor(float) {};
};

#endif
