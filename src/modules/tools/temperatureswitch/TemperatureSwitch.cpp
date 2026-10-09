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

#include "TemperatureSwitch.h"
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "modules/tools/temperaturecontrol/TemperatureControlPublicAccess.h"
#include "SwitchPublicAccess.h"
#include "LaserPublicAccess.h"

#include "utils.h"
#include "Gcode.h"
#include "Config.h"
#include "ConfigValue.h"
#include "checksumm.h"
#include "PublicData.h"
#include "StreamOutputPool.h"
#include "TemperatureControlPool.h"
#include "mri.h"
#include "modules/tools/accessories/BedCleaning.h"
#include "modules/tools/accessories/SpindleAccessories.h"

#define temperatureswitch_checksum                      CHECKSUM("temperatureswitch")
#define enable_checksum                                 CHECKSUM("enable")
#define temperatureswitch_threshold_temp_checksum 	    CHECKSUM("threshold_temp")
#define temperatureswitch_cooldown_power_init_checksum  CHECKSUM("cooldown_power_init")
#define temperatureswitch_cooldown_power_step_checksum  CHECKSUM("cooldown_power_step")
#define temperatureswitch_cooldown_power_laser_checksum CHECKSUM("cooldown_power_laser")
#define temperatureswitch_cooldown_delay_checksum       CHECKSUM("cooldown_delay")

#define temperatureswitch_switch_checksum               CHECKSUM("switch")
#define designator_checksum                             CHECKSUM("designator")
#define temperature_source_checksum                     CHECKSUM("temperature_source")

TemperatureSwitch::TemperatureSwitch()
{
}

TemperatureSwitch::~TemperatureSwitch()
{
    THEKERNEL->unregister_for_event(ON_SECOND_TICK, this);
    THEKERNEL->unregister_for_event(ON_GCODE_RECEIVED, this);
}

// Load module
void TemperatureSwitch::on_module_loaded()
{
    vector<uint16_t> modulist;
    // allow for multiple temperature switches
    THEKERNEL->config->get_module_list(&modulist, temperatureswitch_checksum);
    for (auto m : modulist) {
        load_config(m);
    }

    // no longer need this instance as it is just used to load the other instances
    delete this;
}

TemperatureSwitch* TemperatureSwitch::load_config(uint16_t modcs)
{
    // see if enabled
    if (!THEKERNEL->config->value(temperatureswitch_checksum, modcs, enable_checksum)->as_bool(false)) {
        return nullptr;
    }

    // load settings from config file
    string switchname = THEKERNEL->config->value(temperatureswitch_checksum, modcs, temperatureswitch_switch_checksum)->as_string("");
    if(switchname.empty()) {
		// no switch specified so invalid entry
		THEKERNEL->streams->printf("WARNING TEMPERATURESWITCH: no switch specified\n");
		return nullptr;
    }

    // create a new temperature switch module
    TemperatureSwitch *ts= new TemperatureSwitch();

    ts->temperatureswitch_switch_cs= get_checksum(switchname); // checksum of the switch to use

    ts->temperatureswitch_threshold_temp = THEKERNEL->config->value(temperatureswitch_checksum, modcs, temperatureswitch_threshold_temp_checksum)->as_number(35.0f);
    ts->temperatureswitch_cooldown_power_init = THEKERNEL->config->value(temperatureswitch_checksum, modcs, temperatureswitch_cooldown_power_init_checksum)->as_number(50.0f);
    ts->temperatureswitch_cooldown_power_step = THEKERNEL->config->value(temperatureswitch_checksum, modcs, temperatureswitch_cooldown_power_step_checksum)->as_number(10.0f);
    ts->temperatureswitch_cooldown_power_laser = THEKERNEL->config->value(temperatureswitch_checksum, modcs, temperatureswitch_cooldown_power_laser_checksum)->as_number(80.0f);
    ts->temperatureswitch_cooldown_delay = THEKERNEL->config->value(temperatureswitch_checksum, modcs, temperatureswitch_cooldown_delay_checksum)->as_number(180);
    const std::string source = THEKERNEL->config->value(
        temperatureswitch_checksum, modcs, temperature_source_checksum)->as_string("");
    if(source == "highest") {
        ts->temperature_source = 2;
    } else if(source == "module") {
        ts->temperature_source = 1;
    } else if(CARVERA == THEKERNEL->factory_set->MachineModel) {
        ts->temperature_source = 2;
    } else if(CARVERA_AIR == THEKERNEL->factory_set->MachineModel) {
        ts->temperature_source = 1;
    } else {
        ts->temperature_source = 0;
    }

    ts->fan_mode = FanMode::idle;
    ts->cooldown_elapsed_s = 0;

    ts->register_for_event(ON_SECOND_TICK);
    
    ts->module_checksum = modcs;

    return ts;
}

void TemperatureSwitch::on_gcode_received(void *argument)
{
}

// Called once a second but we only need to service on the cooldown and heatup poll intervals
void TemperatureSwitch::on_second_tick(void *argument)
{
	bool ok;
	if (THEKERNEL->get_laser_mode()) {
		
	    if(CARVERA == THEKERNEL->factory_set->MachineModel)
	    {
			if (fan_mode != FanMode::laser)
	    		THEKERNEL->streams->printf("Laser on, Turn on spindle fan...\r\n");
	    	struct pad_switch pad;
	    	pad.state = true;
	    	pad.value = temperatureswitch_cooldown_power_laser;
		    ok = PublicData::set_value(switch_checksum, this->temperatureswitch_switch_cs, state_value_checksum, &pad);
		    if (!ok) {
		        THEKERNEL->streams->printf("Error turn on spindle fan.\r\n");
		    }
		    fan_mode = FanMode::laser;
		    cooldown_elapsed_s = 0;
		}
	} else {
		float current_temp = 0;
		
		if(this->temperature_source == 2) {
	    	current_temp = this->get_highest_temperature();
	    } else if(this->temperature_source == 1) {
	    	current_temp = this->get_temperature();
	    }

		struct pad_switch current_switch{};
		PublicData::get_value(switch_checksum, this->temperatureswitch_switch_cs, 0, &current_switch);
		float accessory_power = current_switch.manual_value;
		float requested_power = 0;
		bool power_requested = accessory_power > 0;
        if (THEKERNEL->bed_cleaning->requested_fan_power(this->temperatureswitch_switch_cs, requested_power) ||
            THEKERNEL->spindle_accessories->requested_fan_power(this->temperatureswitch_switch_cs,
                                                                 requested_power)) {
			accessory_power = fmaxf(accessory_power, requested_power);
			power_requested = true;
		}
		if (power_requested) {
            float power = accessory_power;
            if(current_temp >= this->temperatureswitch_threshold_temp) {
                power = fmaxf(power, this->temperatureswitch_cooldown_power_init +
                    (current_temp - this->temperatureswitch_threshold_temp) *
                    this->temperatureswitch_cooldown_power_step);
            }
            struct pad_switch pad{};
            pad.state = true;
            pad.value = fminf(power, 100.0F);
            PublicData::set_value(
                switch_checksum, this->temperatureswitch_switch_cs,
                state_value_checksum, &pad);
            fan_mode = FanMode::accessory;
            cooldown_elapsed_s = 0;
            return;
        }
	    if (current_temp >= this->temperatureswitch_threshold_temp) {
	    	struct pad_switch pad;
	    	pad.state = true;
	    	pad.value = temperatureswitch_cooldown_power_init + (current_temp - temperatureswitch_threshold_temp) * temperatureswitch_cooldown_power_step;
		    ok = PublicData::set_value(switch_checksum, this->temperatureswitch_switch_cs, state_value_checksum, &pad);
		    if (!ok) {
		    	if(CARVERA == THEKERNEL->factory_set->MachineModel)
	    		{
		        	THEKERNEL->streams->printf("Error turn on spindle fan.\r\n");
		        }
			    else if(CARVERA_AIR == THEKERNEL->factory_set->MachineModel)
			    {
			    	THEKERNEL->streams->printf("Error turn on fan.\r\n");
			    }
		        
		    }
			fan_mode = FanMode::thermal;
			cooldown_elapsed_s = 0;
	    } else {
            if (fan_mode == FanMode::accessory) {
                bool switch_state = false;
                PublicData::set_value(
                    switch_checksum, this->temperatureswitch_switch_cs,
                    state_checksum, &switch_state);
                fan_mode = FanMode::idle;
                cooldown_elapsed_s = 0;
            } else if (fan_mode == FanMode::laser || fan_mode == FanMode::thermal) {
				fan_mode = FanMode::cooldown;
				cooldown_elapsed_s = 0;
			} else if (fan_mode == FanMode::cooldown) {
				cooldown_elapsed_s++;
				if (cooldown_elapsed_s > temperatureswitch_cooldown_delay) {
//	    			if (!THEKERNEL->is_uploading())
//	    				THEKERNEL->streams->printf("Spindle temp: [%.2f], Turn off spindle fan...\r\n", current_temp);
	    			bool switch_state = false;
	    		    ok = PublicData::set_value(switch_checksum, this->temperatureswitch_switch_cs, state_checksum, &switch_state);
	    		    if (!ok) {
				    	if(CARVERA == THEKERNEL->factory_set->MachineModel)
			    		{
	    		        	THEKERNEL->streams->printf("Error turn off spindle fan.\r\n");
	    		        }
					    else if(CARVERA_AIR == THEKERNEL->factory_set->MachineModel)
					    {
					    	THEKERNEL->streams->printf("Error turn off fan.\r\n");
					    }
	    		    }
					fan_mode = FanMode::idle;
					cooldown_elapsed_s = 0;
	    		}
	    	}
		}
	}
}

float TemperatureSwitch::get_temperature()
{
    std::vector<struct pad_temperature> controllers;
    bool ok = PublicData::get_value(temperature_control_checksum, poll_controls_checksum, &controllers);
    if (ok) {
        for (auto &c : controllers) {
        	if (c.id == this->module_checksum) {
        		return c.current_temperature;
        	}
        }
    }
    return 50;
}

// Get the highest temperature from the set of temperature controllers
float TemperatureSwitch::get_highest_temperature()
{
    float high_temp = 0.0;

    std::vector<struct pad_temperature> controllers;
    bool ok = PublicData::get_value(temperature_control_checksum, poll_controls_checksum, &controllers);
    if (ok) {
        for (auto &c : controllers) {
            // check if this controller's temp is the highest and save it if so
            if (c.current_temperature > high_temp) {
                high_temp = c.current_temperature;
            }
        }
    }
    return high_temp;
}
