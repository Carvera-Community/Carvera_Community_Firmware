#include "libs/Kernel.h"
#include "MainButton.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "Config.h"
#include "SlowTicker.h"
#include "libs/SerialMessage.h"
#include "libs/StreamOutput.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "StreamOutputPool.h"
#include "us_ticker_api.h"
#include "EndstopsPublicAccess.h"
#include "PlayerPublicAccess.h"
#include "SwitchPublicAccess.h"
#include "libs/PublicData.h"
#include "PublicDataRequest.h"
#include "MainButtonPublicAccess.h"
#include "TemperatureControlPublicAccess.h"
#include "LaserPublicAccess.h"
#include "PlayerPublicAccess.h"
#include "ATCHandlerPublicAccess.h"
#include "Gcode.h"
#include "modules/robot/Conveyor.h"

using namespace std;

#define main_button_enable_checksum 				CHECKSUM("main_button_enable")
#define main_button_pin_checksum    				CHECKSUM("main_button_pin")
#define main_button_LED_R_pin_checksum    			CHECKSUM("main_button_LED_R_pin")
#define main_button_LED_G_pin_checksum    			CHECKSUM("main_button_LED_G_pin")
#define main_button_LED_B_pin_checksum    			CHECKSUM("main_button_LED_B_pin")
#define main_button_poll_frequency_checksum			CHECKSUM("main_button_poll_frequency")
#define main_long_press_time_ms_checksum			CHECKSUM("main_button_long_press_time")
#define main_button_long_press_checksum				CHECKSUM("main_button_long_press_enable")

#define e_stop_pin_checksum							CHECKSUM("e_stop_pin")
#define ps12_pin_checksum							CHECKSUM("ps12_pin")
#define ps24_pin_checksum							CHECKSUM("ps24_pin")
#define power_fan_delay_s_checksum					CHECKSUM("power_fan_delay_s")

#define power_checksum								CHECKSUM("power")
#define auto_sleep_checksum							CHECKSUM("auto_sleep")
#define auto_sleep_min_checksum						CHECKSUM("auto_sleep_min")
#define turn_off_min_checksum						CHECKSUM("turn_off_min")
#define stop_on_cover_open_checksum					CHECKSUM("stop_on_cover_open")

#define sd_ok_checksum								CHECKSUM("sd_ok")


MainButton::MainButton()
{
	this->sd_ok = false;
	this->using_12v = false;
	this->led_update_timer = 0;
	this->hold_toggle = 0;
    this->button_state = NONE;
    this->button_pressed = false;
    this->stop_on_cover_open = false;
    this->sleep_countdown_us = us_ticker_read();
    this->light_countdown_us = us_ticker_read();
    this->power_fan_countdown_us = us_ticker_read();
    this->old_state = IDLE;
}

void MainButton::on_module_loaded()
{
    bool main_button_enable = THEKERNEL->config->value( main_button_enable_checksum )->by_default(true)->as_bool(); // @deprecated
    if (!main_button_enable) {
        delete this;
        return;
    }
	
    this->main_button_LED_R.from_string( THEKERNEL->config->value( main_button_LED_R_pin_checksum )->by_default("1.10")->as_string())->as_output();
    this->main_button_LED_G.from_string( THEKERNEL->config->value( main_button_LED_G_pin_checksum )->by_default("1.15")->as_string())->as_output();
    this->main_button_LED_B.from_string( THEKERNEL->config->value( main_button_LED_B_pin_checksum )->by_default("1.14")->as_string())->as_output();
    
    if(CARVERA == THEKERNEL->factory_set->MachineModel)
    {
	    this->main_button.from_string( THEKERNEL->config->value( main_button_pin_checksum )->by_default("1.16^")->as_string())->as_input();
	}
	else if(CARVERA_AIR == THEKERNEL->factory_set->MachineModel)
    {
    	this->main_button.from_string( THEKERNEL->config->value( main_button_pin_checksum )->by_default("2.13!^")->as_string())->as_input();
    }
    this->poll_frequency = THEKERNEL->config->value( main_button_poll_frequency_checksum )->by_default(20)->as_number();
    this->long_press_time_ms = THEKERNEL->config->value( main_long_press_time_ms_checksum )->by_default(3000)->as_number();
    this->long_press_enable = THEKERNEL->config->value( main_button_long_press_checksum )->by_default(false)->as_string();
	if(CARVERA == THEKERNEL->factory_set->MachineModel)
    {
    	this->e_stop.from_string( THEKERNEL->config->value( e_stop_pin_checksum )->by_default("0.26^")->as_string())->as_input();
    }
	else if(CARVERA_AIR == THEKERNEL->factory_set->MachineModel)
    {
    	this->e_stop.from_string( THEKERNEL->config->value( e_stop_pin_checksum )->by_default("0.20^")->as_string())->as_input();
    }
    this->PS12.from_string( THEKERNEL->config->value( ps12_pin_checksum )->by_default("0.22")->as_string())->as_output();
    this->PS24.from_string( THEKERNEL->config->value( ps24_pin_checksum )->by_default("0.10")->as_string())->as_output();
    this->power_fan_delay_s = THEKERNEL->config->value( power_fan_delay_s_checksum )->by_default(30)->as_int();

    this->auto_sleep = THEKERNEL->config->value(power_checksum, auto_sleep_checksum )->by_default(true)->as_bool();
    this->auto_sleep_min = THEKERNEL->config->value(power_checksum, auto_sleep_min_checksum )->by_default(30)->as_number();


    this->enable_light = THEKERNEL->config->value(get_checksum("switch"), get_checksum("light"), get_checksum("startup_state"))->by_default(false)->as_bool();
    this->turn_off_light_min = THEKERNEL->config->value(light_checksum, turn_off_min_checksum )->by_default(10)->as_number();

    this->stop_on_cover_open = THEKERNEL->config->value( stop_on_cover_open_checksum )->by_default(false)->as_bool(); // @deprecated

    this->sd_ok = THEKERNEL->config->value( sd_ok_checksum )->by_default(false)->as_bool(); // @deprecated

    this->register_for_event(ON_IDLE);
    this->register_for_event(ON_SECOND_TICK);
    this->register_for_event(ON_GET_PUBLIC_DATA);
    this->register_for_event(ON_SET_PUBLIC_DATA);

    // turn on power
    this->switch_power_12(1);
    this->switch_power_24(1);
	
	if(CARVERA == THEKERNEL->factory_set->MachineModel)
    {
	    this->main_button_LED_R.set(0);
	    this->main_button_LED_G.set(0);
	    this->main_button_LED_B.set(0);
	}
	else if(CARVERA_AIR == THEKERNEL->factory_set->MachineModel)
    {
    	this->set_led_colors(0, 0, 0);
    	THEKERNEL->slow_ticker->attach( 4, this, &MainButton::led_tick );
    }

    THEKERNEL->slow_ticker->attach( this->poll_frequency, this, &MainButton::button_tick );
}

void MainButton::switch_power_12(int state)
{
	this->PS12.set(state);
}

void MainButton::switch_power_24(int state)
{
	this->PS24.set(state);
}

void MainButton::on_second_tick(void *)
{
    // check if sd card is ok
	if (!this->sd_ok && !THEKERNEL->is_halted()) {
        THEKERNEL->set_halt_reason(SD_ERROR);
        THEKERNEL->call_event(ON_HALT, nullptr);
	}

	bool vacuum_on = false;
	bool toolsensor_on = false;
    // get switchs state
    struct pad_switch pad;
    if (PublicData::get_value(switch_checksum, get_checksum("vacuum"), 0, &pad)) {
    	vacuum_on = pad.state;
    }

    if (PublicData::get_value(switch_checksum, get_checksum("toolsensor"), 0, &pad)) {
    	toolsensor_on = pad.state;
    }

	// check if 12v is being used
	if (THEKERNEL->get_laser_mode() || vacuum_on || toolsensor_on) {
		using_12v = true;
	} else {
		using_12v = false;
	}
}

void MainButton::on_idle(void *argument)
{
	bool cover_open_stop = false;
	bool e_stop_pressed = this->e_stop.get();
    if (e_stop_pressed || button_state == BUTTON_LED_UPDATE || button_state == BUTTON_SHORT_PRESSED || button_state == BUTTON_LONG_PRESSED) {
    	// get current status
    	uint8_t state = THEKERNEL->get_state();
    	if (e_stop_pressed && state != ALARM) {
    	    THEKERNEL->set_halt_reason(E_STOP);
    	    THEKERNEL->call_event(ON_HALT, nullptr);
    	}
		if (this->stop_on_cover_open && !THEKERNEL->is_halted()) {
            void *return_value;
			bool cover_endstop_state;
            bool ok = PublicData::get_value( player_checksum, is_playing_checksum, &return_value );
            if (ok) {
                bool playing = *static_cast<bool *>(return_value);
                if (playing) {
                	ok = PublicData::get_value(endstops_checksum, get_cover_endstop_state_checksum, 0, &cover_endstop_state);
					if (ok) {
						if (!cover_endstop_state) {
							cover_open_stop = true;
						}
					}
                }
            }
		}
		// turn on/off power fan with delay
		if ((state == IDLE || state == SLEEP) && !using_12v) {
    		// reset sleep timer
    		if (us_ticker_read() - this->power_fan_countdown_us > (uint32_t)this->power_fan_delay_s * 1000000) {
				// turn off 12v
				if(CARVERA == THEKERNEL->factory_set->MachineModel)
			    {
	    			this->switch_power_12(0);
	    		}
    		}
		} else {
			this->switch_power_12(1);
			this->power_fan_countdown_us = us_ticker_read();
		}
		// auto sleep method
    	if (this->auto_sleep && auto_sleep_min > 0) {
        	if (state == IDLE) {
        		// reset sleep timer
        		if (us_ticker_read() - sleep_countdown_us > (uint32_t)auto_sleep_min * 60 * 1000000) {
    				// turn off 12V/24V power supply
					this->switch_power_12(0);
					this->switch_power_24(0);// turn off light
					bool b = false;
					PublicData::set_value( switch_checksum, light_checksum, state_checksum, &b );
        			// go to sleep
    				THEKERNEL->set_sleeping(true);
    				THEKERNEL->call_event(ON_HALT, nullptr);
        		}
        	} else {
        		sleep_countdown_us = us_ticker_read();
        	}
    	}
    	if (this->enable_light && turn_off_light_min > 0) {
        	if (state == IDLE) {
        		// turn off light timer
        		if (us_ticker_read() - light_countdown_us > (uint32_t)turn_off_light_min * 60 * 1000000) {
        			light_countdown_us = us_ticker_read();
        			// turn off light
					bool b = false;
					PublicData::set_value( switch_checksum, light_checksum, state_checksum, &b );
        		}
        	} else if (state != SLEEP) {
        		light_countdown_us = us_ticker_read();
        		// turn on the light
        		struct pad_switch pad;
        		bool ok = false;
        		ok = PublicData::get_value(switch_checksum, light_checksum, state_checksum, &pad);
        		if (ok) {
        			if(!(bool)pad.value)
        			{
						bool b = true;
						PublicData::set_value( switch_checksum, light_checksum, state_checksum, &b );
        			}
        		}
        	}
    	}
    	uint8_t halt_reason;
    	if (button_state == BUTTON_SHORT_PRESSED) {
    		switch (state) {
    			case IDLE:
    			case RUN:
    			case HOME:
    				// Halt
    		        THEKERNEL->set_halt_reason(MANUAL);
    		        THEKERNEL->call_event(ON_HALT, nullptr);
    				break;
    			case HOLD:
    				// resume
    				THEKERNEL->set_feed_hold(false);
    				break;
    			case ALARM:
    				// do nothing
    				break;
    			case SLEEP:
    				// reset
    				system_reset(false);
    				break;
    			case TOOL:
					// Finish tool change waiting for Carvera Air
					THEKERNEL->set_tool_waiting(false);
    				break;
    		}
    	} else if (button_state == BUTTON_LONG_PRESSED ) {
    		switch (state) {
    			case IDLE:
    				if (this->long_press_enable == "Repeat" ) {
	    				// restart last job (if there is)
	    			    PublicData::set_value( player_checksum, restart_job_checksum, NULL);
	    			}
	    			else if(this->long_press_enable == "Sleep" ) {
	    				// turn off 12V/24V power supply
						this->switch_power_12(0);
						this->switch_power_24(0);
	        			// go to sleep
	    				THEKERNEL->set_sleeping(true);
	    				THEKERNEL->call_event(ON_HALT, nullptr);
	    			}
					else if(this->long_press_enable == "ToolChange" && !THEKERNEL->is_tool_waiting()) {
						uint8_t atc_clamp_status;
						uint8_t old_state = state;
						THEKERNEL->set_tool_waiting(true);

						PublicData::get_value(atc_handler_checksum, get_atc_clamped_status_checksum, 0, &atc_clamp_status);
						THEKERNEL->streams->printf("atc clamped status = %d \n" , atc_clamp_status); //0 is unhomed, 1 is clamped, 2 is unclamped
						THEKERNEL->streams->printf("Running Manual Tool Change From Front Button\n");
						Gcode gc1("M490.2", &StreamOutput::NullStream);
						Gcode gc2("M490.1", &StreamOutput::NullStream);
						switch (atc_clamp_status){
							case 0: //atc unhomed
								THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc1);
								THECONVEYOR->wait_for_idle();
								THEKERNEL->streams->printf("Toolholder Should Be Empty\n");
								break;

							case 1: //atc clamped
								THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc1);
								THECONVEYOR->wait_for_idle();
								THEKERNEL->streams->printf("Toolholder Should Be Empty\n");
								break;
							case 2: //atc unclamped
								THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc2);
								THECONVEYOR->wait_for_idle();
								THEKERNEL->streams->printf("Tool Should Be Clamped. Set Tool Number\n");
								break;
							default:
								break;

						}
						THECONVEYOR->wait_for_idle();
						THEKERNEL->set_tool_waiting(false);
						
	    			}

// turn off 12V/24V power supply
//    				this->switch_power_12(0);
//    				this->switch_power_24(0);
//    				// sleep
//    				THEKERNEL->set_sleeping(true);
//    				THEKERNEL->call_event(ON_HALT, nullptr);
    				break;
    			case RUN:
    			case HOME:
    				// halt
    		        THEKERNEL->set_halt_reason(MANUAL);
    		        THEKERNEL->call_event(ON_HALT, nullptr);
    				break;
    			case HOLD:
    				// resume
					if(this->long_press_enable == "ToolChange" && !THEKERNEL->is_tool_waiting()) {
						uint8_t atc_clamp_status;
						uint8_t old_state = state;
						THEKERNEL->set_tool_waiting(true);

						PublicData::get_value(atc_handler_checksum, get_atc_clamped_status_checksum, 0, &atc_clamp_status);
						THEKERNEL->streams->printf("atc clamped status = %d \n" , atc_clamp_status); //0 is unhomed, 1 is clamped, 2 is unclamped
						THEKERNEL->streams->printf("Running Manual Tool Change From Front Button\n");
						Gcode gc1("M490.2", &StreamOutput::NullStream);
						Gcode gc2("M490.1", &StreamOutput::NullStream);
						switch (atc_clamp_status){
							case 0: //atc unhomed
								THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc1);
								THECONVEYOR->wait_for_idle();
								THEKERNEL->streams->printf("Toolholder Should Be Empty\n");
								break;

							case 1: //atc clamped
								THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc1);
								THECONVEYOR->wait_for_idle();
								THEKERNEL->streams->printf("Toolholder Should Be Empty\n");
								break;
							case 2: //atc unclamped
								THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc2);
								THECONVEYOR->wait_for_idle();
								THEKERNEL->streams->printf("Tool Should Be Clamped. Set Tool Number\n");
								break;
							default:
								break;

						}
						THECONVEYOR->wait_for_idle();
						THEKERNEL->set_tool_waiting(false);
						
	    			}else
					{
						THEKERNEL->set_feed_hold(false);
						THEKERNEL->set_suspending(false);
					}

    				
    				break;
    			case ALARM:
    				halt_reason = THEKERNEL->get_halt_reason();
    				if (halt_reason > 20) {
    					// reset
        				system_reset(false);
    				} else {
    					// unlock
    		            THEKERNEL->call_event(ON_HALT, (void *)1); // clears on_halt
    		            THEKERNEL->streams->printf("UnKill button pressed, Halt cleared\r\n");
    				}
    				break;
    			case SLEEP:
    				// reset
    				system_reset(false);
    				break;
    		}
    	} else {
    		// update led status
		    if(CARVERA == THEKERNEL->factory_set->MachineModel)
		    {
	    		switch (state) {
	    			case IDLE:
	    			    this->main_button_LED_R.set(0);
	    			    this->main_button_LED_G.set(0);
	    			    this->main_button_LED_B.set(1);
	    				break;
	    			case RUN:
	    			    this->main_button_LED_R.set(0);
	    			    this->main_button_LED_G.set(1);
	    			    this->main_button_LED_B.set(0);
	    				break;
	    			case HOME:
	    			    this->main_button_LED_R.set(1);
	    			    this->main_button_LED_G.set(1);
	    			    this->main_button_LED_B.set(0);
	    				break;
	    			case HOLD:
	    				this->hold_toggle ++;
	    			    this->main_button_LED_R.set(0);
	    			    this->main_button_LED_G.set(this->hold_toggle % 4  < 2 ? 1 : 0);
	    			    this->main_button_LED_B.set(0);
	    				break;
	    			case ALARM:
	    			    this->main_button_LED_R.set(1);
	    			    this->main_button_LED_G.set(0);
	    			    this->main_button_LED_B.set(0);
	    			    break;
	    			case SLEEP:
	    			    this->main_button_LED_R.set(1);
	    			    this->main_button_LED_G.set(1);
	    			    this->main_button_LED_B.set(1);
	    				break;
	    			case SUSPEND:
	    				this->hold_toggle ++;
	    			    this->main_button_LED_R.set(0);
	    			    this->main_button_LED_G.set(0);
	    			    this->main_button_LED_B.set(this->hold_toggle % 4  < 2 ? 1 : 0);
	    				break;
	    			case WAIT:
	    				this->hold_toggle ++;
	    			    this->main_button_LED_R.set(this->hold_toggle % 4  < 2 ? 1 : 0);
	    			    this->main_button_LED_G.set(this->hold_toggle % 4  < 2 ? 1 : 0);
	    			    this->main_button_LED_B.set(0);
	    				break;
	    		}

	    	}
/*			else if(CARVERA_AIR == THEKERNEL->factory_set->MachineModel)
		    {
		    	if (state != old_state) 
		    	{
	    			old_state = state;
	        		switch (state) {
	        			case IDLE:
	        				this->set_led_colors(0, 0, 100);
	        				break;
	        			case RUN:
	        				this->set_led_colors(0, 100, 0);
	        				break;
	        			case HOME:
	        				this->set_led_colors(100, 20, 0);
	        				break;
	        			case ALARM:
	        				this->set_led_colors(100, 0, 0);
	        			    break;
	        			case SLEEP:
	        				this->set_led_colors(100, 100, 100);
	        				break;
	        		}
	        	}

		    }*/
    		if (cover_open_stop) {
		        THEKERNEL->set_halt_reason(COVER_OPEN);
		        THEKERNEL->call_event(ON_HALT, nullptr);
    		}
    	}
    	button_state = NONE;
    }
}

// Check the state of the button and act accordingly using the following FSM
// Note this is ISR so don't do anything nasty in here
// If in toggle mode (locking estop) then button down will kill, and button up will unkill if unkill is enabled
// otherwise it will look for a 2 second press on the kill button to unkill if unkill is set
uint32_t MainButton::button_tick(uint32_t dummy)
{
	if (this->main_button.get()) {
		if (!this->button_pressed) {
			// button down
			this->button_pressed = true;
			this->button_press_time = us_ticker_read();
		}
		if (us_ticker_read() - this->button_press_time > this->long_press_time_ms * 1000) {
			this->hold_toggle ++;
		} else {
			this->hold_toggle = 0;
		}
		if(CARVERA == THEKERNEL->factory_set->MachineModel){
			this->main_button_LED_R.set(this->hold_toggle % 4  < 2 ? 0 : 1);
		} else if(CARVERA_AIR == THEKERNEL->factory_set->MachineModel){
			switch (THEKERNEL->get_state()) {
				case IDLE:
					this->set_led_colors(this->hold_toggle % 4  < 2 ? 0 : 100, 0, 1);
					break;
				case RUN:
					this->set_led_colors(this->hold_toggle % 4  < 2 ? 104 : 0, 100, 0);
					break;
				case HOME:
					this->set_led_colors(this->hold_toggle % 4  < 2 ? 104 : 0, 20, 0);
					break;
				case ALARM:
					this->set_led_colors(this->hold_toggle % 4  < 2 ? 104 : 0, 0, 0);
					break;
				case SLEEP:
					this->set_led_colors(this->hold_toggle % 4  < 2 ? 104 : 0, 100, 100);
					break;
				}
			}
	} else {
		// button up
		if (this->button_pressed) {
			if (us_ticker_read() - this->button_press_time > this->long_press_time_ms * 1000) {
				button_state = BUTTON_LONG_PRESSED;
				this->hold_toggle = 0;
			} else {
				button_state = BUTTON_SHORT_PRESSED;
			}
			this->button_pressed = false;
		} else {
            if(++led_update_timer > this->poll_frequency * 0.2) {
            	button_state = BUTTON_LED_UPDATE;
            	led_update_timer = 0;
            }
		}
	}
    return 0;
}

void MainButton::on_get_public_data(void* argument)
{
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);

    if (pdr->starts_with(main_button_checksum)) {
    	if (pdr->second_element_is(get_e_stop_state_checksum)) {
			char *data = static_cast<char *>(pdr->get_data_ptr());
			// e-stop status
			data[0] = (char)this->e_stop.get();
			pdr->set_taken();
    	}
    }
}

void MainButton::on_set_public_data(void* argument)
{
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);

    if (pdr->starts_with(main_button_checksum)) {
    	if (pdr->second_element_is(switch_power_12_checksum)) {
			char *state = static_cast<char *>(pdr->get_data_ptr());
    		this->switch_power_12(*state);
    	}
    	if (pdr->second_element_is(switch_power_24_checksum)) {
			char *state = static_cast<char *>(pdr->get_data_ptr());
    		this->switch_power_24(*state);
    	}
    }
}
uint32_t MainButton::led_tick(uint32_t dummy)
{
	uint8_t state = THEKERNEL->get_state();
	switch (state) {
		case HOLD:
			this->hold_toggle ++;
			this->set_led_colors(0, this->hold_toggle % 4  < 2 ? 104 : 0, 0);
			break;
		case SUSPEND:
			this->hold_toggle ++;
			this->set_led_colors(0, 0, this->hold_toggle % 4 < 2 ? 104 : 0);
			break;
		case WAIT:
			this->hold_toggle ++;
			this->set_led_colors(this->hold_toggle % 4  < 2 ? 104 : 0, this->hold_toggle % 4 <2 ? 24 : 0, 0);
			break;
		case TOOL:
			this->hold_toggle ++;
			struct tool_status tool;
    		PublicData::get_value( atc_handler_checksum, get_tool_status_checksum, &tool );
    		switch(tool.target_tool)
    		{
    			case 1:
    				if(this->hold_toggle % 5 == 0)
						this->set_led_num(0,104,104,0,0,0,1);
					if(this->hold_toggle % 5 == 1)
						this->set_led_colors(0, 0, 0);
					if(this->hold_toggle % 5 == 2)
						this->set_led_colors(0, 0, 0);
					if(this->hold_toggle % 5 == 3)
						this->set_led_colors(0, 0, 0);
					if(this->hold_toggle % 5 == 4)
						this->set_led_colors(0, 0, 0);
    				break;
    			case 2:
    				if(this->hold_toggle % 5 == 0)
						this->set_led_num(0,104,104,0,0,0,2);
					if(this->hold_toggle % 5 == 1)
						this->set_led_colors(0, 0, 0);
					if(this->hold_toggle % 5 == 2)
						this->set_led_colors(0, 0, 0);
					if(this->hold_toggle % 5 == 3)
						this->set_led_colors(0, 0, 0);
					if(this->hold_toggle % 5 == 4)
						this->set_led_colors(0, 0, 0);
    				break;
    			case 3:
    				if(this->hold_toggle % 5 == 0)
						this->set_led_num(0,104,104,0,0,0,3);
					if(this->hold_toggle % 5 == 1)
						this->set_led_colors(0, 0, 0);
					if(this->hold_toggle % 5 == 2)
						this->set_led_colors(0, 0, 0);
					if(this->hold_toggle % 5 == 3)
						this->set_led_colors(0, 0, 0);
					if(this->hold_toggle % 5 == 4)
						this->set_led_colors(0, 0, 0);
    				break;
    			case 4:
    				if(this->hold_toggle % 5 == 0)
						this->set_led_num(0,104,104,0,0,0,3);
					if(this->hold_toggle % 5 == 1)
						this->set_led_colors(0, 0, 0);
					if(this->hold_toggle % 5 == 2)
						this->set_led_num(0,104,104,0,0,0,1);
					if(this->hold_toggle % 5 == 3)
						this->set_led_colors(0, 0, 0);
					if(this->hold_toggle % 5 == 4)
						this->set_led_colors(0, 0, 0);
    				break;
    			case 5:
    				if(this->hold_toggle % 5 == 0)
						this->set_led_num(0,104,104,0,0,0,3);
					if(this->hold_toggle % 5 == 1)
						this->set_led_colors(0, 0, 0);
					if(this->hold_toggle % 5 == 2)
						this->set_led_num(0,104,104,0,0,0,2);
					if(this->hold_toggle % 5 == 3)
						this->set_led_colors(0, 0, 0);
					if(this->hold_toggle % 5 == 4)
						this->set_led_colors(0, 0, 0);
    				break;
    			case 6:
    				if(this->hold_toggle % 5 == 0)
						this->set_led_num(0,104,104,0,0,0,3);
					if(this->hold_toggle % 5 == 1)
						this->set_led_colors(0, 0, 0);
					if(this->hold_toggle % 5 == 2)
						this->set_led_num(0,104,104,0,0,0,3);
					if(this->hold_toggle % 5 == 3)
						this->set_led_colors(0, 0, 0);
					if(this->hold_toggle % 5 == 4)
						this->set_led_colors(0, 0, 0);
    				break;
    			default:
    				if(this->hold_toggle % 5 == 0)
						this->set_led_colors(0, 104, 104);
					if(this->hold_toggle % 5 == 1)
						this->set_led_colors(0, 0, 0);
					if(this->hold_toggle % 5 == 2)
						this->set_led_colors(0, 104, 104);
					if(this->hold_toggle % 5 == 3)
						this->set_led_colors(0, 0, 0);
					if(this->hold_toggle % 5 == 4)
						this->set_led_colors(0, 0, 0);
    				break;
    			
    		}
			
			break;
	}
	if (state != old_state) 
	{
		old_state = state;
		switch (state) {
			case IDLE:
				this->set_led_colors(0, 0, 104);
				break;
			case RUN:
				this->set_led_colors(0, 104, 0);
				break;
			case HOME:
				this->set_led_colors(104, 24, 0);
				break;
			case ALARM:
				this->set_led_colors(104, 0, 0);
			    break;
			case SLEEP:
				this->set_led_colors(104, 104, 104);
				break;
		}
	}
	else if((RUN == state) && (true == THEKERNEL->checkled) )
	{
		this->hold_toggle ++;
		if(this->hold_toggle % 4 == 0)
		{
			this->set_led_colors(104, 0 , 0);
		}
	}
	return 0;
}

void MainButton::set_led_color(unsigned char R1, unsigned char G1, unsigned char B1,unsigned char R2, unsigned char G2, unsigned char B2,unsigned char R3, unsigned char G3, unsigned char B3,unsigned char R4, unsigned char G4, unsigned char B4,unsigned char R5, unsigned char G5, unsigned char B5)
{
	unsigned char temp, j;
	
	//first segment
	temp = R1;
	for (j = 0; j < 8; j++) {
		if (temp & (0x80 >> j)) //����1
		{
			LPC_GPIO1->FIOSET = 1 << 15; //0x00008000;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			LPC_GPIO1->FIOCLR = 1 << 15;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		}
		else                //����0
		{
			LPC_GPIO1->FIOSET = 1 << 15;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();
			LPC_GPIO1->FIOCLR = 1 << 15;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		}
	}
	temp = G1;
	for (j = 0; j < 8; j++) {
		if (temp & (0x80 >> j)) //����1
		{
			LPC_GPIO1->FIOSET = 1 << 15; //0x00008000;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			LPC_GPIO1->FIOCLR = 1 << 15;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		}
		else                //����0
		{
			LPC_GPIO1->FIOSET = 1 << 15;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();
			LPC_GPIO1->FIOCLR = 1 << 15;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		}
	}
	temp = B1;
	for (j = 0; j < 8; j++) {
		if (temp & (0x80 >> j)) //����1
		{
			LPC_GPIO1->FIOSET = 1 << 15; //0x00008000;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			LPC_GPIO1->FIOCLR = 1 << 15;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		}
		else                //����0
		{
			LPC_GPIO1->FIOSET = 1 << 15;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();
			LPC_GPIO1->FIOCLR = 1 << 15;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		}
	}
	//second segment
	temp = R2;
	for (j = 0; j < 8; j++) {
		if (temp & (0x80 >> j)) //����1
		{
			LPC_GPIO1->FIOSET = 1 << 15; //0x00008000;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			LPC_GPIO1->FIOCLR = 1 << 15;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		}
		else                //����0
		{
			LPC_GPIO1->FIOSET = 1 << 15;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();
			LPC_GPIO1->FIOCLR = 1 << 15;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		}
	}
	temp = G2;
	for (j = 0; j < 8; j++) {
		if (temp & (0x80 >> j)) //����1
		{
			LPC_GPIO1->FIOSET = 1 << 15; //0x00008000;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			LPC_GPIO1->FIOCLR = 1 << 15;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		}
		else                //����0
		{
			LPC_GPIO1->FIOSET = 1 << 15;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();
			LPC_GPIO1->FIOCLR = 1 << 15;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		}
	}
	temp = B2;
	for (j = 0; j < 8; j++) {
		if (temp & (0x80 >> j)) //����1
		{
			LPC_GPIO1->FIOSET = 1 << 15; //0x00008000;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			LPC_GPIO1->FIOCLR = 1 << 15;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		}
		else                //����0
		{
			LPC_GPIO1->FIOSET = 1 << 15;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();
			LPC_GPIO1->FIOCLR = 1 << 15;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		}
	}
	//third segment
	temp = R3;
	for (j = 0; j < 8; j++) {
		if (temp & (0x80 >> j)) //����1
		{
			LPC_GPIO1->FIOSET = 1 << 15; //0x00008000;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			LPC_GPIO1->FIOCLR = 1 << 15;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		}
		else                //����0
		{
			LPC_GPIO1->FIOSET = 1 << 15;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();
			LPC_GPIO1->FIOCLR = 1 << 15;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		}
	}
	temp = G3;
	for (j = 0; j < 8; j++) {
		if (temp & (0x80 >> j)) //����1
		{
			LPC_GPIO1->FIOSET = 1 << 15; //0x00008000;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			LPC_GPIO1->FIOCLR = 1 << 15;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		}
		else                //����0
		{
			LPC_GPIO1->FIOSET = 1 << 15;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();
			LPC_GPIO1->FIOCLR = 1 << 15;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		}
	}
	temp = B3;
	for (j = 0; j < 8; j++) {
		if (temp & (0x80 >> j)) //����1
		{
			LPC_GPIO1->FIOSET = 1 << 15; //0x00008000;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			LPC_GPIO1->FIOCLR = 1 << 15;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		}
		else                //����0
		{
			LPC_GPIO1->FIOSET = 1 << 15;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();
			LPC_GPIO1->FIOCLR = 1 << 15;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		}
	}
	//forth segment
	temp = R4;
	for (j = 0; j < 8; j++) {
		if (temp & (0x80 >> j)) //����1
		{
			LPC_GPIO1->FIOSET = 1 << 15; //0x00008000;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			LPC_GPIO1->FIOCLR = 1 << 15;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		}
		else                //����0
		{
			LPC_GPIO1->FIOSET = 1 << 15;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();
			LPC_GPIO1->FIOCLR = 1 << 15;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		}
	}
	temp = G4;
	for (j = 0; j < 8; j++) {
		if (temp & (0x80 >> j)) //����1
		{
			LPC_GPIO1->FIOSET = 1 << 15; //0x00008000;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			LPC_GPIO1->FIOCLR = 1 << 15;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		}
		else                //����0
		{
			LPC_GPIO1->FIOSET = 1 << 15;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();
			LPC_GPIO1->FIOCLR = 1 << 15;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		}
	}
	temp = B4;
	for (j = 0; j < 8; j++) {
		if (temp & (0x80 >> j)) //����1
		{
			LPC_GPIO1->FIOSET = 1 << 15; //0x00008000;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			LPC_GPIO1->FIOCLR = 1 << 15;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		}
		else                //����0
		{
			LPC_GPIO1->FIOSET = 1 << 15;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();
			LPC_GPIO1->FIOCLR = 1 << 15;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		}
	}
	
	//fifth segment
	temp = R5;
	for (j = 0; j < 8; j++) {
		if (temp & (0x80 >> j)) //����1
		{
			LPC_GPIO1->FIOSET = 1 << 15; //0x00008000;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			LPC_GPIO1->FIOCLR = 1 << 15;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		}
		else                //����0
		{
			LPC_GPIO1->FIOSET = 1 << 15;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();
			LPC_GPIO1->FIOCLR = 1 << 15;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		}
	}
	temp = G5;
	for (j = 0; j < 8; j++) {
		if (temp & (0x80 >> j)) //����1
		{
			LPC_GPIO1->FIOSET = 1 << 15; //0x00008000;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			LPC_GPIO1->FIOCLR = 1 << 15;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		}
		else                //����0
		{
			LPC_GPIO1->FIOSET = 1 << 15;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();
			LPC_GPIO1->FIOCLR = 1 << 15;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		}
	}
	temp = B5;
	for (j = 0; j < 8; j++) {
		if (temp & (0x80 >> j)) //����1
		{
			LPC_GPIO1->FIOSET = 1 << 15; //0x00008000;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			LPC_GPIO1->FIOCLR = 1 << 15;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		}
		else                //����0
		{
			LPC_GPIO1->FIOSET = 1 << 15;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();
			LPC_GPIO1->FIOCLR = 1 << 15;
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		}
	}
}

/*
void MainButton::set_led_color(unsigned char R, unsigned char G, unsigned char B)
{
	unsigned char i, j, temp[3];
	temp[0] = R;
	temp[1] = G;
	temp[2] = B;
	for (i = 0; i < 3; i++) {
		for (j = 0; j < 8; j++) {
			if (temp[i] & (0x80 >> j)) //����1
			{
				LPC_GPIO1->FIOSET = 1 << 15; //0x00008000;
				__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
				__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
				__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
				__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
				__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
				__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
				__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
				__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
				__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
				__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
				LPC_GPIO1->FIOCLR = 1 << 15;
				__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			}
			else                //����0
			{
				LPC_GPIO1->FIOSET = 1 << 15;
				__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
				__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
				__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
				LPC_GPIO1->FIOCLR = 1 << 15;
				__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
				__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
				__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
				__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
				__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
				__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
				__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
				__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
				__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			}
		}
	}
}
*/

void MainButton::set_led_colors(unsigned char R, unsigned char G, unsigned char B)
{
    //__disable_irq();
    // stop TIMER0 and TIMER1 for save time
	NVIC_DisableIRQ(TIMER0_IRQn);
	NVIC_DisableIRQ(TIMER1_IRQn);
	set_led_color(R, G, B, R, G, B, R, G, B, R, G, B, R, G, B);
//	set_led_color(R, G, B);
//	set_led_color(R, G, B);
//	set_led_color(R, G, B);
//	set_led_color(R, G, B);
    //__enable_irq();
    NVIC_EnableIRQ(TIMER0_IRQn);     // Enable interrupt handler
	NVIC_EnableIRQ(TIMER1_IRQn);     // Enable interrupt handler
}

void MainButton::set_led_num(unsigned char ColorFR, unsigned char ColorFG, unsigned char ColorFB, unsigned char ColorBR, unsigned char ColorBG, unsigned char ColorBB, unsigned char num)
{
    __disable_irq();
    switch(num)
    {
    	case 1:
    		set_led_color(ColorFR, ColorFG, ColorFB, ColorBR, ColorBG, ColorBB, ColorBR, ColorBG, ColorBB, ColorBR, ColorBG, ColorBB, ColorBR, ColorBG, ColorBB);
    		//set_led_color(ColorBR, ColorBG, ColorBB);
    		//set_led_color(ColorBR, ColorBG, ColorBB);
    		//set_led_color(ColorBR, ColorBG, ColorBB);
    		//set_led_color(ColorBR, ColorBG, ColorBB);
    		break;	
    	case 2:
    		set_led_color(ColorFR, ColorFG, ColorFB, ColorBR, ColorBG, ColorBB, ColorFR, ColorFG, ColorFB, ColorBR, ColorBG, ColorBB, ColorBR, ColorBG, ColorBB);
    		//set_led_color(ColorBR, ColorBG, ColorBB);
    		//set_led_color(ColorFR, ColorFG, ColorFB);
    		//set_led_color(ColorBR, ColorBG, ColorBB);
    		//set_led_color(ColorBR, ColorBG, ColorBB);
    		break;	
    	case 3:
    		set_led_color(ColorFR, ColorFG, ColorFB, ColorBR, ColorBG, ColorBB, ColorFR, ColorFG, ColorFB, ColorBR, ColorBG, ColorBB, ColorFR, ColorFG, ColorFB);
    		//set_led_color(ColorBR, ColorBG, ColorBB);
    		//set_led_color(ColorFR, ColorFG, ColorFB);
    		//set_led_color(ColorBR, ColorBG, ColorBB);
    		//set_led_color(ColorFR, ColorFG, ColorFB);
    		break;	
    	default:
    		break;
    }
    __enable_irq();
}
