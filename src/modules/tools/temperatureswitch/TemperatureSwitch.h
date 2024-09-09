/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

/*
TemperatureSwitch is an optional module that will automatically turn on or off a switch
based on a setpoint temperature. It is commonly used to turn on/off a cooling fan or water pump
to cool the hot end's cold zone. Specifically, it turns one of the small MOSFETs on or off.

Author: Michael Hackney, mhackney@eclecticangler.com
*/

#ifndef TEMPERATURESWITCH_MODULE_H
#define TEMPERATURESWITCH_MODULE_H

using namespace std;

#include "libs/Module.h"
#include <string>
#include <vector>

class TemperatureSwitch : public Module
{
    public:
        TemperatureSwitch();
        ~TemperatureSwitch();
        void on_module_loaded();
        void on_second_tick(void *argument);
        void on_gcode_received(void *argument);
        TemperatureSwitch* load_config(uint16_t modcs);


    private:
        // get the highest temperature from the set of configured temperature controllers
        float get_highest_temperature();

        float temperatureswitch_threshold_temp;
        float temperatureswitch_cooldown_power_init;
        float temperatureswitch_cooldown_power_step;
        float temperatureswitch_cooldown_power_laser;
        uint16_t temperatureswitch_cooldown_delay;

        // temperatureswitch.hotend.switch
        uint16_t temperatureswitch_switch_cs;


        // our internal second counter
        int cooldown_delay_counter;


};

#endif
