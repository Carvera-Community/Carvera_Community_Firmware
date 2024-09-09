/*
    This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
    Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
    Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
    You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Laser.h"
#include "Module.h"
#include "Kernel.h"
#include "nuts_bolts.h"
#include "Config.h"
#include "StreamOutputPool.h"
#include "SerialMessage.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "StepTicker.h"
#include "Block.h"
#include "SlowTicker.h"
#include "Robot.h"
#include "utils.h"
#include "Pin.h"
#include "Gcode.h"
#include "PwmOut.h" // mbed.h lib
#include "Conveyor.h"

#include "libs/PublicData.h"
#include "PublicDataRequest.h"
#include "LaserPublicAccess.h"
#include "SwitchPublicAccess.h"



#include <algorithm>

#define laser_module_enable_checksum            CHECKSUM("laser_module_enable")
#define laser_module_pin_checksum               CHECKSUM("laser_module_pin")
#define laser_module_pwm_pin_checksum           CHECKSUM("laser_module_pwm_pin")
#define laser_module_ttl_pin_checksum           CHECKSUM("laser_module_ttl_pin")
#define laser_module_pwm_period_checksum        CHECKSUM("laser_module_pwm_period")
#define laser_module_test_power_checksum        CHECKSUM("laser_module_test_power")
#define laser_module_maximum_power_checksum     CHECKSUM("laser_module_maximum_power")
#define laser_module_minimum_power_checksum     CHECKSUM("laser_module_minimum_power")
#define laser_module_max_power_checksum         CHECKSUM("laser_module_max_power")
#define laser_module_maximum_s_value_checksum   CHECKSUM("laser_module_maximum_s_value")

Laser::Laser()
{
    laser_on = false;
    scale = 1;
    testing = false;
}

void Laser::on_module_loaded()
{
    if( !THEKERNEL->config->value( laser_module_enable_checksum )->by_default(true)->as_bool() ) {
        // as not needed free up resource
        delete this;
        return;
    }

    // Get smoothie-style pin from config
    this->laser_pin = new Pin();
    this->laser_pin->from_string(THEKERNEL->config->value(laser_module_pin_checksum)->by_default("2.12")->as_string())->as_output();
    if (!this->laser_pin->connected()) {
        delete this->laser_pin;
        this->laser_pin= nullptr;
        delete this;
        return;
	}

	Pin *dummy_pin = new Pin();
	dummy_pin->from_string(THEKERNEL->config->value(laser_module_pwm_pin_checksum)->by_default("2.4")->as_string())->as_output();
    pwm_pin = dummy_pin->hardware_pwm();
    if (pwm_pin == NULL) {
        THEKERNEL->streams->printf("Error: Laser cannot use P%d.%d (P2.0 - P2.5, P1.18, P1.20, P1.21, P1.23, P1.24, P1.26, P3.25, P3.26 only). Laser module disabled.\n", dummy_pin->port_number, dummy_pin->pin);
        delete dummy_pin;
        delete this;
        return;
    }


    this->pwm_inverting = dummy_pin->is_inverting();

    delete dummy_pin;
    dummy_pin = NULL;

    // TTL settings
    this->ttl_pin = new Pin();
    ttl_pin->from_string( THEKERNEL->config->value(laser_module_ttl_pin_checksum)->by_default("nc" )->as_string())->as_output();
    this->ttl_used = ttl_pin->connected();
    this->ttl_inverting = ttl_pin->is_inverting();
    if (ttl_used) {
        ttl_pin->set(0);
    } else {
        delete ttl_pin;
        ttl_pin = NULL;
    }


    uint32_t period = THEKERNEL->config->value(laser_module_pwm_period_checksum)->by_default(20)->as_number();
    this->pwm_pin->period_us(period);
    this->pwm_pin->write(this->pwm_inverting ? 1 : 0);
    this->laser_test_power = THEKERNEL->config->value(laser_module_test_power_checksum)->by_default(0.1f)->as_number() ;
    this->laser_maximum_power = THEKERNEL->config->value(laser_module_maximum_power_checksum)->by_default(1.0f)->as_number() ;
    this->laser_minimum_power = THEKERNEL->config->value(laser_module_minimum_power_checksum)->by_default(0)->as_number() ;

    // S value that represents maximum (default 1)
    this->laser_maximum_s_value = THEKERNEL->config->value(laser_module_maximum_s_value_checksum)->by_default(1.0f)->as_number() ;

    set_laser_power(0);

    //register for events
    this->register_for_event(ON_HALT);
    this->register_for_event(ON_GCODE_RECEIVED);
    this->register_for_event(ON_CONSOLE_LINE_RECEIVED);
    this->register_for_event(ON_GET_PUBLIC_DATA);

    // no point in updating the power more than the PWM frequency, but not faster than 1KHz
    ms_per_tick = 1000 / std::min(1000UL, 1000000 / period);
    // 2024
    THEKERNEL->slow_ticker->attach(std::min(1000UL, 1000000 / period), this, &Laser::set_proportional_power);
    // THEKERNEL->slow_ticker->attach(std::min(4000UL, 1000000 / period), this, &Laser::set_proportional_power);
    // THEKERNEL->slow_ticker->attach(1, this, &Laser::set_proportional_power);

}

void Laser::on_console_line_received( void *argument )
{
    if(THEKERNEL->is_halted()) return; // if in halted state ignore any commands

    SerialMessage *msgp = static_cast<SerialMessage *>(argument);
    string possible_command = msgp->message;

    // ignore anything that is not lowercase or a letter
    if(possible_command.empty() || !islower(possible_command[0]) || !isalpha(possible_command[0])) {
        return;
    }

    string cmd = shift_parameter(possible_command);

    // Act depending on command
    if (cmd == "laser") {
        string laser_cmd = shift_parameter(possible_command);
        if (laser_cmd.empty()) {
        	THEKERNEL->streams->printf("Usage: laser on|off|status|test|testoff\n");
            return;
        }
        if (laser_cmd == "on") {
        	THEKERNEL->set_laser_mode(true);
        	// turn on laser pin
        	this->laser_pin->set(true);
        	THEKERNEL->streams->printf("turning laser mode on\n");
        } else if (laser_cmd == "off") {
        	THEKERNEL->set_laser_mode(false);
        	this->laser_pin->set(false);
        	this->testing = false;
        	this->set_laser_power(0);
        	// turn off laser pin
        	THEKERNEL->streams->printf("turning laser mode off and return to CNC mode\n");
        } else if (laser_cmd == "status") {
        	THEKERNEL->streams->printf("laser mode state: %s\n", THEKERNEL->get_laser_mode() ? "on" : "off");
        } else if (laser_cmd == "test" && THEKERNEL->get_laser_mode()) {
        	this->testing = true;
        }
    }
}

// returns instance
void Laser::on_get_public_data(void* argument)
{
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);
    if(!pdr->starts_with(laser_checksum)) return;
    if(pdr->second_element_is(get_laser_status_checksum)) {
		// ok this is targeted at us, so set the requ3sted data in the pointer passed into us
		struct laser_status *t= static_cast<laser_status*>(pdr->get_data_ptr());
		t->mode = THEKERNEL->get_laser_mode();
		t->state = this->laser_on;
		t->testing = this->testing;
	    float p = pwm_pin->read();
	    t->power = (this->pwm_inverting ? 1 - p : p) * 100;
		t->scale = this->scale * 100;
		pdr->set_taken();
    }

}


void Laser::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);

    // M codes execute immediately
    if (gcode->has_m) {
    	if (gcode->m == 3 && THEKERNEL->get_laser_mode())
		{
    		THECONVEYOR->wait_for_idle();
            // M3 with S value provided: set speed
            if (gcode->has_letter('S'))
            {
            	THEROBOT->set_s_value(gcode->get_value('S'));
            }
    		this->laser_on = true;
    		this->testing = false;
    		// THEKERNEL->streams->printf("Laser on, S: %1.4f\n", THEROBOT->get_s_value());
		} else if (gcode->m == 5) {
    		THECONVEYOR->wait_for_idle();
			this->laser_on = false;
			this->testing = false;
		} else if (gcode->m == 321 && !THEKERNEL->get_laser_mode()) { // change to laser mode
			THECONVEYOR->wait_for_idle();
        	THEKERNEL->set_laser_mode(true);
        	// turn on laser pin
        	this->laser_pin->set(true);
        	if (gcode->subcode == 2) {
            	THEKERNEL->streams->printf("turning laser mode on\n");
        	} else {
            	char buf[32];
            	// drop current tool
                int n = snprintf(buf, sizeof(buf), "M6T-1");
                string g1(buf, n);
                Gcode gc1(g1, &(StreamOutput::NullStream));
                THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc1);
            	// change g92 offset
                n = snprintf(buf, sizeof(buf), "G92.5Z0");
                string g2(buf, n);
                Gcode gc2(g2, &(StreamOutput::NullStream));
                THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc2);

            	THEKERNEL->streams->printf("turning laser mode on and change offset\n");
        	}

        } else if (gcode->m == 322) { // change to CNC mode
        	THECONVEYOR->wait_for_idle();
        	THEKERNEL->set_laser_mode(false);
        	this->laser_pin->set(false);
        	this->testing = false;
        	if (gcode->subcode == 2) {
            	THEKERNEL->streams->printf("turning laser mode off and return to CNC mode\n");
        	} else {
            	// change g92 offset
                char buf[32];
                int n = snprintf(buf, sizeof(buf), "G92.1");
                string g(buf, n);
                Gcode gc(g, &(StreamOutput::NullStream));
                THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc);

            	// turn off laser pin
            	THEKERNEL->streams->printf("turning laser mode off and restore offset\n");
        	}
        } else if (gcode->m == 323) {
        	this->testing = true;
			// turn on test mode
        	THEKERNEL->streams->printf("turning laser test mode on\n");
        } else if (gcode->m == 324) {
        	this->testing = false;
			// turn off test mode
        	THEKERNEL->streams->printf("turning laser test mode off\n");
        } else if (gcode->m == 325) { // M223 S100 change laser power by percentage S
            if(gcode->has_letter('S')) {
                this->scale = gcode->get_value('S') / 100.0F;
            } else {
            	THEKERNEL->streams->printf("Laser power scale at %6.2f %%\n", this->scale * 100.0F);
            }
        }
    }
}

// calculates the current speed ratio from the currently executing block
float Laser::current_speed_ratio(const Block *block) const
{
    // find the primary moving actuator (the one with the most steps)

	// 2024
    size_t pm = 0;
    uint32_t max_steps = 0;
    for (size_t i = 0; i < THEROBOT->get_number_registered_motors(); i++) {
        // find the motor with the most steps
        if(block->steps[i] > max_steps) {
            max_steps = block->steps[i];
            pm = i;
        }
    }

    // figure out the ratio of its speed, from 0 to 1 based on where it is on the trapezoid,
    // this is based on the fraction it is of the requested rate (nominal rate)
    float ratio = block->get_trapezoid_rate(pm) / block->nominal_rate;

    return ratio;

    /*
    // find the primary moving actuator (the one with the most steps)
    size_t pm = block->move_axis;
    // figure out the ratio of its speed, from 0 to 1 based on where it is on the trapezoid,
    // this is based on the fraction it is of the requested rate (nominal rate)
    float ratio = block->get_trapezoid_rate(pm) / block->nominal_rate;

    return ratio;
    */

}

// get laser power for the currently executing block, returns false if nothing running or a G0
bool Laser::get_laser_power(float& power) const
{
    const Block *block = StepTicker::getInstance()->get_current_block();

    // Note to avoid a race condition where the block is being cleared we check the is_ready flag which gets cleared first,
    // as this is an interrupt if that flag is not clear then it cannot be cleared while this is running and the block will still be valid (albeit it may have finished)
    if (block != nullptr && block->is_ready && block->is_g123) {
    	// 2024
        float requested_power = (float)block->s_value / (1 << 11) / this->laser_maximum_s_value; // s_value is 1.11 Fixed point
        float ratio = current_speed_ratio(block);
        power = requested_power * ratio * scale;
        return true;

        /*
		int axis = block->move_axis;
		int shift_steps = block->tick_info[axis].steps_to_move / int(block->s_count);
		int idx = block->tick_info[axis].step_count / shift_steps;

        float requested_power = (float)block->s_values[idx] / (1 << 11) / this->laser_maximum_s_value; // s_value is 1.11 Fixed point
        float ratio = current_speed_ratio(block);
        power = requested_power * ratio * scale;
        return true;
        */

    }

    return false;
}

// called every millisecond from timer ISR
uint32_t Laser::set_proportional_power(uint32_t dummy)
{
	if (!THEKERNEL->get_laser_mode()) {
		return 0;
	}
    if (this->testing) {
        set_laser_power(this->laser_test_power * scale);
        return 0;
    }

    if (laser_on) {
        float power;
        if(get_laser_power(power)) {
            // adjust power to maximum power and actual velocity
            float proportional_power = ( (this->laser_maximum_power - this->laser_minimum_power) * power ) + this->laser_minimum_power;
            set_laser_power(proportional_power);

        } else {
            // turn laser off
            set_laser_power(0);
        }
    } else {
        // turn laser off
        set_laser_power(0);
    }

    return 0;
}

bool Laser::set_laser_power(float power)
{
    // Ensure power is >=0 and <= 1
    power = confine(power, 0.0F, 1.0F);

    if(power > 0.0001F) {
        this->pwm_pin->write(this->pwm_inverting ? 1 - power : power);
        if(!laser_on && this->ttl_used) this->ttl_pin->set(true);
        return true;
    } else {
        this->pwm_pin->write(this->pwm_inverting ? 1 : 0);
        if (this->ttl_used) this->ttl_pin->set(false);
        return false;
    }
}

void Laser::on_halt(void *argument)
{
    if(argument == nullptr) {
        set_laser_power(0);
        this->laser_on = false;
    	THEKERNEL->set_laser_mode(false);
    	this->laser_pin->set(false);
    	this->testing = false;
    	THEROBOT->clearLaserOffset();
    }
}
