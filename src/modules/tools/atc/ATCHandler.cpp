/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "ATCHandler.h"

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "ATCHandler.h"
#include "SlowTicker.h"
#include "Tool.h"
#include "PublicDataRequest.h"
#include "Config.h"
#include "StepperMotor.h"
#include "Robot.h"
#include "ConfigValue.h"
#include "Conveyor.h"
#include "checksumm.h"
#include "PublicData.h"
#include "Gcode.h"
#include "modules/robot/Conveyor.h"
#include "libs/StreamOutputPool.h"
#include "libs/StreamOutput.h"
#include "SwitchPublicAccess.h"
#include "libs/utils.h"

#include "libs/SerialMessage.h"
#include "libs/StreamOutput.h"
#include "modules/utils/player/PlayerPublicAccess.h"
#include "ATCHandlerPublicAccess.h"
#include "ZProbePublicAccess.h"
#include "SpindlePublicAccess.h"

#include "us_ticker_api.h"

#include "FileStream.h"
#include <math.h>

#define ATC_AXIS 4
#define STEPPER THEROBOT->actuators
// #define STEPS_PER_MM(a) (STEPPER[a]->get_steps_per_mm())

#define atc_checksum            	CHECKSUM("atc")
#define probe_checksum            	CHECKSUM("probe")
#define endstop_pin_checksum      	CHECKSUM("homing_endstop_pin")
#define debounce_ms_checksum      	CHECKSUM("homing_debounce_ms")
#define max_travel_mm_checksum    	CHECKSUM("homing_max_travel_mm")
#define homing_retract_mm_checksum  CHECKSUM("homing_retract_mm")
#define homing_rate_mm_s_checksum   CHECKSUM("homing_rate_mm_s")
#define action_mm_checksum      	CHECKSUM("action_mm")
#define action_rate_mm_s_checksum   CHECKSUM("action_rate_mm_s")

#define detector_switch_checksum    CHECKSUM("toolsensor")
#define detector_checksum           CHECKSUM("detector")
#define detect_pin_checksum			CHECKSUM("detect_pin")
#define detect_rate_mm_s_checksum	CHECKSUM("detect_rate_mm_s")
#define detect_travel_mm_checksum 	CHECKSUM("detect_travel_mm")

#define safe_z_checksum				CHECKSUM("safe_z_mm")
#define safe_z_empty_checksum		CHECKSUM("safe_z_empty_mm")
#define safe_z_offset_checksum		CHECKSUM("safe_z_offset_mm")
#define fast_z_rate_checksum		CHECKSUM("fast_z_rate_mm_m")
#define slow_z_rate_checksum		CHECKSUM("slow_z_rate_mm_m")
#define margin_rate_checksum		CHECKSUM("margin_rate_mm_m")

#define fast_rate_mm_m_checksum		CHECKSUM("fast_rate_mm_m")
#define slow_rate_mm_m_checksum		CHECKSUM("slow_rate_mm_m")
#define retract_mm_checksum			CHECKSUM("retract_mm")
#define probe_height_mm_checksum	CHECKSUM("probe_height_mm")

#define coordinate_checksum			CHECKSUM("coordinate")
#define anchor1_x_checksum			CHECKSUM("anchor1_x")
#define anchor1_y_checksum			CHECKSUM("anchor1_y")
#define anchor2_offset_x_checksum	CHECKSUM("anchor2_offset_x")
#define anchor2_offset_y_checksum	CHECKSUM("anchor2_offset_y")
#define rotation_offset_x_checksum	CHECKSUM("rotation_offset_x")
#define rotation_offset_y_checksum	CHECKSUM("rotation_offset_y")
#define rotation_offset_z_checksum	CHECKSUM("rotation_offset_z")
#define toolrack_offset_x_checksum	CHECKSUM("toolrack_offset_x")
#define toolrack_offset_y_checksum	CHECKSUM("toolrack_offset_y")
#define toolrack_z_checksum			CHECKSUM("toolrack_z")
#define clearance_x_checksum		CHECKSUM("clearance_x")
#define clearance_y_checksum		CHECKSUM("clearance_y")
#define clearance_z_checksum		CHECKSUM("clearance_z")

ATCHandler::ATCHandler()
{
    atc_status = NONE;
    atc_home_info.clamp_status = UNHOMED;
    atc_home_info.triggered = false;
    detector_info.triggered = false;
    ref_tool_mz = 0.0;
    cur_tool_mz = 0.0;
    tool_offset = 0.0;
    last_pos[0] = 0.0;
    last_pos[1] = 0.0;
    last_pos[2] = 0.0;
    probe_laser_last = 9999;
    playing_file = false;
    tool_number = 6;
    g28_triggered = false;
    goto_position = -1;
    position_x = 8888;
    position_y = 8888;
}

void ATCHandler::clear_script_queue(){
	while (!this->script_queue.empty()) {
		this->script_queue.pop();
	}
}

void ATCHandler::fill_drop_scripts(int old_tool) {
	char buff[100];
	struct atc_tool *current_tool = &atc_tools[old_tool];
	// set atc status
	this->script_queue.push("M497.1");
    // lift z axis to atc start position
	snprintf(buff, sizeof(buff), "G53 G0 Z%.3f", THEROBOT->from_millimeters(this->clearance_z));
	this->script_queue.push(buff);
    // move x and y to active tool position
	snprintf(buff, sizeof(buff), "G53 G0 X%.3f Y%.3f", THEROBOT->from_millimeters(current_tool->mx_mm), THEROBOT->from_millimeters(current_tool->my_mm));
	this->script_queue.push(buff);
	// move around to see if tool rack is empty
	this->script_queue.push("M492.2");
    // move x and y to reseted tool position
	snprintf(buff, sizeof(buff), "G53 G0 X%.3f Y%.3f", THEROBOT->from_millimeters(current_tool->mx_mm), THEROBOT->from_millimeters(current_tool->my_mm));
	this->script_queue.push(buff);
    // drop z axis to z position with fast speed
	snprintf(buff, sizeof(buff), "G53 G1 Z%.3f F%.3f", THEROBOT->from_millimeters(current_tool->mz_mm + safe_z_offset_mm), THEROBOT->from_millimeters(fast_z_rate));
	this->script_queue.push(buff);
    // drop z axis with slow speed
	snprintf(buff, sizeof(buff), "G53 G1 Z%.3f F%.3f", THEROBOT->from_millimeters(current_tool->mz_mm), THEROBOT->from_millimeters(slow_z_rate));
	this->script_queue.push(buff);
	// loose tool
	this->script_queue.push("M490.2");
	// lift z to safe position with fast speed
	snprintf(buff, sizeof(buff), "G53 G0 Z%.3f", THEROBOT->from_millimeters(this->safe_z_empty_mm));
	this->script_queue.push(buff);
	// set new tool to -1
	this->script_queue.push("M493.2 T-1");
	// move around to see if tool is dropped, halt if not
	this->script_queue.push("M492.1");
}

void ATCHandler::fill_pick_scripts(int new_tool, bool clear_z) {
	char buff[100];
	struct atc_tool *current_tool = &atc_tools[new_tool];
	// set atc status
	this->script_queue.push("M497.2");
	// lift z to safe position with fast speed
	snprintf(buff, sizeof(buff), "G53 G0 Z%.3f", THEROBOT->from_millimeters(clear_z ? this->clearance_z : this->safe_z_empty_mm));
	this->script_queue.push(buff);
	// move x and y to new tool position
	snprintf(buff, sizeof(buff), "G53 G0 X%.3f Y%.3f", THEROBOT->from_millimeters(current_tool->mx_mm), THEROBOT->from_millimeters(current_tool->my_mm));
	this->script_queue.push(buff);
	// move around to see if tool rack is filled
	this->script_queue.push("M492.1");
	// loose tool
	this->script_queue.push("M490.2");
	// move x and y to reseted tool position
	snprintf(buff, sizeof(buff), "G53 G0 X%.3f Y%.3f", THEROBOT->from_millimeters(current_tool->mx_mm), THEROBOT->from_millimeters(current_tool->my_mm));
	this->script_queue.push(buff);
    // drop z axis to z position with fast speed
	snprintf(buff, sizeof(buff), "G53 G1 Z%.3f F%.3f", THEROBOT->from_millimeters(current_tool->mz_mm + safe_z_offset_mm), THEROBOT->from_millimeters(fast_z_rate));
	this->script_queue.push(buff);
    // drop z axis with slow speed
	snprintf(buff, sizeof(buff), "G53 G1 Z%.3f F%.3f", THEROBOT->from_millimeters(current_tool->mz_mm), THEROBOT->from_millimeters(slow_z_rate));
	this->script_queue.push(buff);
	// clamp tool
	this->script_queue.push("M490.1");
	// lift z to safe position with fast speed
	snprintf(buff, sizeof(buff), "G53 G0 Z%.3f", THEROBOT->from_millimeters(this->safe_z_mm));
	this->script_queue.push(buff);
	// move around to see if tool rack is empty, halt if not
	this->script_queue.push("M492.2");
	// set new tool
	snprintf(buff, sizeof(buff), "M493.2 T%d", new_tool);
	this->script_queue.push(buff);

}

void ATCHandler::fill_cali_scripts(bool is_probe, bool clear_z) {
	char buff[100];
	// set atc status
	this->script_queue.push("M497.3");
	// clamp tool if in laser mode
	if (THEKERNEL->get_laser_mode()) {
		this->script_queue.push("M490.1");
	}
	// lift z to safe position with fast speed
	snprintf(buff, sizeof(buff), "G53 G0 Z%.3f", THEROBOT->from_millimeters(clear_z ? this->clearance_z : this->safe_z_mm));
	this->script_queue.push(buff);
	// move x and y to calibrate position
	snprintf(buff, sizeof(buff), "G53 G0 X%.3f Y%.3f", THEROBOT->from_millimeters(probe_mx_mm), THEROBOT->from_millimeters(probe_my_mm));
	this->script_queue.push(buff);
	// do calibrate with fast speed
	snprintf(buff, sizeof(buff), "G38.6 Z%.3f F%.3f", probe_mz_mm, probe_fast_rate);
	this->script_queue.push(buff);
	// lift a bit
	snprintf(buff, sizeof(buff), "G91 G0 Z%.3f", THEROBOT->from_millimeters(probe_retract_mm));
	this->script_queue.push(buff);
	// do calibrate with slow speed
	snprintf(buff, sizeof(buff), "G38.6 Z%.3f F%.3f", -1 - probe_retract_mm, probe_slow_rate);
	this->script_queue.push(buff);
	// save new tool offset
	this->script_queue.push("M493.1");
	// lift z to safe position with fast speed
	snprintf(buff, sizeof(buff), "G53 G0 Z%.3f", THEROBOT->from_millimeters(this->safe_z_mm));
	this->script_queue.push(buff);

	// check if wireless probe is will be triggered
	if (is_probe) {
		this->script_queue.push("M492.3");
	}
}

void ATCHandler::fill_margin_scripts(float x_pos, float y_pos, float x_pos_max, float y_pos_max) {
	char buff[100];

	// set atc status
	this->script_queue.push("M497.4");

	// open probe laser
	this->script_queue.push("M494.1");

	// lift z to safe position with fast speed
	snprintf(buff, sizeof(buff), "G53 G0 Z%.3f", THEROBOT->from_millimeters(this->clearance_z));
	this->script_queue.push(buff);

	// goto margin start position
	snprintf(buff, sizeof(buff), "G90 G0 X%.3f Y%.3f", THEROBOT->from_millimeters(x_pos), THEROBOT->from_millimeters(y_pos));
	this->script_queue.push(buff);

	// goto margin top left corner
	snprintf(buff, sizeof(buff), "G90 G1 X%.3f Y%.3f F%.3f", THEROBOT->from_millimeters(x_pos), THEROBOT->from_millimeters(y_pos_max), THEROBOT->from_millimeters(this->margin_rate));
	this->script_queue.push(buff);

	// goto margin top right corner
	snprintf(buff, sizeof(buff), "G90 G1 X%.3f Y%.3f F%.3f", THEROBOT->from_millimeters(x_pos_max), THEROBOT->from_millimeters(y_pos_max), THEROBOT->from_millimeters(this->margin_rate));
	this->script_queue.push(buff);

	// goto margin bottom right corner
	snprintf(buff, sizeof(buff), "G90 G1 X%.3f Y%.3f F%.3f", THEROBOT->from_millimeters(x_pos_max), THEROBOT->from_millimeters(y_pos), THEROBOT->from_millimeters(this->margin_rate));
	this->script_queue.push(buff);

	// goto margin start position
	snprintf(buff, sizeof(buff), "G90 G1 X%.3f Y%.3f F%.3f", THEROBOT->from_millimeters(x_pos), THEROBOT->from_millimeters(y_pos), THEROBOT->from_millimeters(this->margin_rate));
	this->script_queue.push(buff);

	// close probe laser
	this->script_queue.push("M494.2");

}

void ATCHandler::fill_goto_origin_scripts(float x_pos, float y_pos) {
	char buff[100];

	// lift z to clearance position with fast speed
	snprintf(buff, sizeof(buff), "G53 G0 Z%.3f", THEROBOT->from_millimeters(this->clearance_z));
	this->script_queue.push(buff);

	// goto start position
	snprintf(buff, sizeof(buff), "G90 G0 X%.3f Y%.3f", THEROBOT->from_millimeters(x_pos), THEROBOT->from_millimeters(y_pos));
	this->script_queue.push(buff);

}

void ATCHandler::fill_zprobe_scripts(float x_pos, float y_pos, float x_offset, float y_offset) {
	char buff[100];

	// set atc status
	this->script_queue.push("M497.5");

	// lift z to safe position with fast speed
	snprintf(buff, sizeof(buff), "G53 G0 Z%.3f", THEROBOT->from_millimeters(this->clearance_z));
	this->script_queue.push(buff);

	// goto z probe position
	snprintf(buff, sizeof(buff), "G90 G0 X%.3f Y%.3f", THEROBOT->from_millimeters(x_pos + x_offset), THEROBOT->from_millimeters(y_pos + y_offset));
	this->script_queue.push(buff);

	// do probe with fast speed
	snprintf(buff, sizeof(buff), "G38.2 Z%.3f F%.3f", probe_mz_mm, probe_fast_rate);
	this->script_queue.push(buff);

	// lift a bit
	snprintf(buff, sizeof(buff), "G91 G0 Z%.3f", THEROBOT->from_millimeters(probe_retract_mm));
	this->script_queue.push(buff);

	// do calibrate with slow speed
	snprintf(buff, sizeof(buff), "G38.2 Z%.3f F%.3f", -1 - probe_retract_mm, probe_slow_rate);
	this->script_queue.push(buff);

	// set z working coordinate
	snprintf(buff, sizeof(buff), "G10 L20 P0 Z%.3f", THEROBOT->from_millimeters(probe_height_mm));
	this->script_queue.push(buff);

	// retract z a bit
	snprintf(buff, sizeof(buff), "G91 G0 Z%.3f", THEROBOT->from_millimeters(probe_retract_mm));
	this->script_queue.push(buff);
}

void ATCHandler::fill_zprobe_abs_scripts() {
	char buff[100];

	// set atc status
	this->script_queue.push("M497.5");

	// lift z to safe position with fast speed
	snprintf(buff, sizeof(buff), "G53 G0 Z%.3f", THEROBOT->from_millimeters(clearance_z));
	this->script_queue.push(buff);

	// goto z probe position
	snprintf(buff, sizeof(buff), "G53 G0 X%.3f Y%.3f", THEROBOT->from_millimeters(anchor1_x + rotation_offset_x - 3), THEROBOT->from_millimeters(anchor1_y + rotation_offset_y));
	this->script_queue.push(buff);

	// do probe with fast speed
	snprintf(buff, sizeof(buff), "G38.2 Z%.3f F%.3f", probe_mz_mm, probe_fast_rate);
	this->script_queue.push(buff);

	// lift a bit
	snprintf(buff, sizeof(buff), "G91 G0 Z%.3f", THEROBOT->from_millimeters(probe_retract_mm));
	this->script_queue.push(buff);

	// do calibrate with slow speed
	snprintf(buff, sizeof(buff), "G38.2 Z%.3f F%.3f", -1 - probe_retract_mm, probe_slow_rate);
	this->script_queue.push(buff);

	// set z working coordinate
	snprintf(buff, sizeof(buff), "G10 L20 P0 Z%.3f", THEROBOT->from_millimeters(rotation_offset_z));
	this->script_queue.push(buff);

	// retract z a bit
	snprintf(buff, sizeof(buff), "G91 G0 Z%.3f", THEROBOT->from_millimeters(probe_retract_mm));
	this->script_queue.push(buff);
}

void ATCHandler::fill_xyzprobe_scripts(float tool_dia, float probe_height) {
	char buff[100];

	// set atc status
	this->script_queue.push("M497.5");

	// do z probe with slow speed
	snprintf(buff, sizeof(buff), "G38.2 Z%.3f F%.3f", probe_mz_mm, probe_slow_rate);
	this->script_queue.push(buff);

	// set Z origin
	snprintf(buff, sizeof(buff), "G10 L20 P0 Z%.3f", THEROBOT->from_millimeters(probe_height));
	this->script_queue.push(buff);

	// lift a bit
	snprintf(buff, sizeof(buff), "G91 G0 Z%.3f", THEROBOT->from_millimeters(probe_retract_mm));
	this->script_queue.push(buff);

	// do x probe with slow speed
	snprintf(buff, sizeof(buff), "G38.2 X%.3f F%.3f", -35.0, probe_slow_rate);
	this->script_queue.push(buff);

	// set x origin
	snprintf(buff, sizeof(buff), "G10 L20 P0 X%.3f", THEROBOT->from_millimeters(tool_dia / 2));
	this->script_queue.push(buff);

	// move right a little bit
	snprintf(buff, sizeof(buff), "G91 G0 X%.3f", THEROBOT->from_millimeters(5.0));
	this->script_queue.push(buff);

	// do y probe with slow speed
	snprintf(buff, sizeof(buff), "G38.2 Y%.3f F%.3f", -35.0, probe_slow_rate);
	this->script_queue.push(buff);

	// set y origin
	snprintf(buff, sizeof(buff), "G10 L20 P0 Y%.3f", THEROBOT->from_millimeters(tool_dia / 2));
	this->script_queue.push(buff);

	// move forward a little bit
	snprintf(buff, sizeof(buff), "G91 G0 Y%.3f", THEROBOT->from_millimeters(5.0));
	this->script_queue.push(buff);

	// retract z to be above probe
	snprintf(buff, sizeof(buff), "G91 G0 Z%.3f", THEROBOT->from_millimeters(15.0));
	this->script_queue.push(buff);

	// move to XY zero
	snprintf(buff, sizeof(buff), "G91 G0 X%.3f Y%0.3f", THEROBOT->from_millimeters(-5 - tool_dia / 2), THEROBOT->from_millimeters(-5 - tool_dia / 2));
	this->script_queue.push(buff);

}


void ATCHandler::fill_autolevel_scripts(float x_pos, float y_pos,
		float x_size, float y_size, int x_grids, int y_grids, float height)
{
	char buff[100];

	// set atc status
	this->script_queue.push("M497.6");

	// goto x and y path origin
	snprintf(buff, sizeof(buff), "G90 G0 X%.3f Y%.3f", THEROBOT->from_millimeters(x_pos), THEROBOT->from_millimeters(y_pos));
	this->script_queue.push(buff);

	// do auto leveling
	snprintf(buff, sizeof(buff), "G32R1X0Y0A%.3fB%.3fI%dJ%dH%.3f", x_size, y_size, x_grids, y_grids, height);
	this->script_queue.push(buff);
}

void ATCHandler::on_module_loaded()
{

    this->register_for_event(ON_GCODE_RECEIVED);
    this->register_for_event(ON_GET_PUBLIC_DATA);
    this->register_for_event(ON_SET_PUBLIC_DATA);
    this->register_for_event(ON_MAIN_LOOP);
    this->register_for_event(ON_HALT);

    this->on_config_reload(this);

    THEKERNEL->slow_ticker->attach(1000, this, &ATCHandler::read_endstop);
    THEKERNEL->slow_ticker->attach(1000, this, &ATCHandler::read_detector);

    THEKERNEL->slow_ticker->attach(1, this, &ATCHandler::countdown_probe_laser);

    // load data from eeprom
    this->active_tool = THEKERNEL->eeprom_data->TOOL;
    this->ref_tool_mz = THEKERNEL->eeprom_data->REFMZ;
    this->cur_tool_mz = THEKERNEL->eeprom_data->TOOLMZ;
    this->tool_offset = THEKERNEL->eeprom_data->TLO;

}

void ATCHandler::on_config_reload(void *argument)
{
	char buff[10];

	atc_home_info.pin.from_string( THEKERNEL->config->value(atc_checksum, endstop_pin_checksum)->by_default("1.0^" )->as_string())->as_input();
	atc_home_info.debounce_ms    = THEKERNEL->config->value(atc_checksum, debounce_ms_checksum)->by_default(1  )->as_number();
	atc_home_info.max_travel    = THEKERNEL->config->value(atc_checksum, max_travel_mm_checksum)->by_default(8  )->as_number();
	atc_home_info.retract    = THEKERNEL->config->value(atc_checksum, homing_retract_mm_checksum)->by_default(3  )->as_number();
	atc_home_info.action_dist    = THEKERNEL->config->value(atc_checksum, action_mm_checksum)->by_default(1  )->as_number();
	atc_home_info.homing_rate    = THEKERNEL->config->value(atc_checksum, homing_rate_mm_s_checksum)->by_default(1  )->as_number();
	atc_home_info.action_rate    = THEKERNEL->config->value(atc_checksum, action_rate_mm_s_checksum)->by_default(1  )->as_number();

	detector_info.detect_pin.from_string( THEKERNEL->config->value(atc_checksum, detector_checksum, detect_pin_checksum)->by_default("0.20^" )->as_string())->as_input();
	detector_info.detect_rate = THEKERNEL->config->value(atc_checksum, detector_checksum, detect_rate_mm_s_checksum)->by_default(1  )->as_number();
	detector_info.detect_travel = THEKERNEL->config->value(atc_checksum, detector_checksum, detect_travel_mm_checksum)->by_default(1  )->as_number();

	this->safe_z_mm = THEKERNEL->config->value(atc_checksum, safe_z_checksum)->by_default(-10)->as_number();
	this->safe_z_empty_mm = THEKERNEL->config->value(atc_checksum, safe_z_empty_checksum)->by_default(-20)->as_number();
	this->safe_z_offset_mm = THEKERNEL->config->value(atc_checksum, safe_z_offset_checksum)->by_default(10)->as_number();
	this->fast_z_rate = THEKERNEL->config->value(atc_checksum, fast_z_rate_checksum)->by_default(500)->as_number();
	this->slow_z_rate = THEKERNEL->config->value(atc_checksum, slow_z_rate_checksum)->by_default(60)->as_number();
	this->margin_rate = THEKERNEL->config->value(atc_checksum, margin_rate_checksum)->by_default(1000)->as_number();

	this->probe_fast_rate = THEKERNEL->config->value(atc_checksum, probe_checksum, fast_rate_mm_m_checksum)->by_default(300  )->as_number();
	this->probe_slow_rate = THEKERNEL->config->value(atc_checksum, probe_checksum, slow_rate_mm_m_checksum)->by_default(60   )->as_number();
	this->probe_retract_mm = THEKERNEL->config->value(atc_checksum, probe_checksum, retract_mm_checksum)->by_default(2   )->as_number();
	this->probe_height_mm = THEKERNEL->config->value(atc_checksum, probe_checksum, probe_height_mm_checksum)->by_default(0   )->as_number();

	this->anchor1_x = THEKERNEL->config->value(coordinate_checksum, anchor1_x_checksum)->by_default(-359  )->as_number();
	this->anchor1_y = THEKERNEL->config->value(coordinate_checksum, anchor1_y_checksum)->by_default(-234  )->as_number();
	this->anchor2_offset_x = THEKERNEL->config->value(coordinate_checksum, anchor2_offset_x_checksum)->by_default(90  )->as_number();
	this->anchor2_offset_y = THEKERNEL->config->value(coordinate_checksum, anchor2_offset_y_checksum)->by_default(45.65F  )->as_number();

	this->toolrack_z = THEKERNEL->config->value(coordinate_checksum, toolrack_z_checksum)->by_default(-105  )->as_number();
	this->toolrack_offset_x = THEKERNEL->config->value(coordinate_checksum, toolrack_offset_x_checksum)->by_default(356  )->as_number();
	this->toolrack_offset_y = THEKERNEL->config->value(coordinate_checksum, toolrack_offset_y_checksum)->by_default(0  )->as_number();

	atc_tools.clear();
	for (int i = 0; i <=  6; i ++) {
		struct atc_tool tool;
		tool.num = i;
	    // lift z axis to atc start position
		snprintf(buff, sizeof(buff), "tool%d", i);
		tool.mx_mm = this->anchor1_x + this->toolrack_offset_x;
		tool.my_mm = this->anchor1_y + this->toolrack_offset_y + (i == 0 ? 210 : (6 - i) * 30);
		tool.mz_mm = this->toolrack_z;
		atc_tools.push_back(tool);
	}
	probe_mx_mm = this->anchor1_x + this->toolrack_offset_x;
	probe_my_mm = this->anchor1_y + this->toolrack_offset_y + 180;
	probe_mz_mm = this->toolrack_z - 40;

	this->rotation_offset_x = THEKERNEL->config->value(coordinate_checksum, rotation_offset_x_checksum)->by_default(-8  )->as_number();
	this->rotation_offset_y = THEKERNEL->config->value(coordinate_checksum, rotation_offset_y_checksum)->by_default(37.5F  )->as_number();
	this->rotation_offset_z = THEKERNEL->config->value(coordinate_checksum, rotation_offset_z_checksum)->by_default(22.5F  )->as_number();

	this->clearance_x = THEKERNEL->config->value(coordinate_checksum, clearance_x_checksum)->by_default(-75  )->as_number();
	this->clearance_y = THEKERNEL->config->value(coordinate_checksum, clearance_y_checksum)->by_default(-3  )->as_number();
	this->clearance_z = THEKERNEL->config->value(coordinate_checksum, clearance_z_checksum)->by_default(-3  )->as_number();
}

void ATCHandler::on_halt(void* argument)
{
    if (argument == nullptr ) {
        this->atc_status = NONE;
        this->clear_script_queue();
        this->set_inner_playing(false);
        THEKERNEL->set_atc_state(ATC_NONE);
        this->atc_home_info.clamp_status = UNHOMED;
	}
}

// Called every millisecond in an ISR
uint32_t ATCHandler::read_endstop(uint32_t dummy)
{

	if(!atc_homing || atc_home_info.triggered) return 0;

    if(STEPPER[ATC_AXIS]->is_moving()) {
        // if it is moving then we check the probe, and debounce it
        if(atc_home_info.pin.get()) {
            if(debounce < atc_home_info.debounce_ms) {
                debounce++;
            } else {
            	STEPPER[ATC_AXIS]->stop_moving();
            	atc_home_info.triggered = true;
                debounce = 0;
            }

        } else {
            // The endstop was not hit yet
            debounce = 0;
        }
    }

    return 0;
}

// Called every millisecond in an ISR
uint32_t ATCHandler::read_detector(uint32_t dummy)
{

    if(!detecting || detector_info.triggered) return 0;

    if (detector_info.detect_pin.get()) {
    	detector_info.triggered = true;
    }

    return 0;
}

// Called every second in an ISR
uint32_t ATCHandler::countdown_probe_laser(uint32_t dummy)
{
	if (this->probe_laser_last < 120) {
		this->probe_laser_last ++;
		PublicData::set_value(atc_handler_checksum, set_wp_laser_checksum, nullptr);
	}
    return 0;
}

bool ATCHandler::laser_detect() {
    // First wait for the queue to be empty
    THECONVEYOR->wait_for_idle();

    // switch on detector
    bool switch_state = true;
    bool ok = PublicData::set_value(switch_checksum, detector_switch_checksum, state_checksum, &switch_state);
    if (!ok) {
        THEKERNEL->streams->printf("ERROR: Failed switch on detector switch.\r\n");
        return false;
    }

    // move around and check laser detector
    detecting = true;
    detector_info.triggered = false;

	float delta[Y_AXIS + 1];
	for (size_t i = 0; i <= Y_AXIS; i++) delta[i] = 0;
	delta[Y_AXIS]= detector_info.detect_travel / 2;
	THEROBOT->delta_move(delta, detector_info.detect_rate, Y_AXIS + 1);
	// wait for it
	THECONVEYOR->wait_for_idle();
	if(THEKERNEL->is_halted()) return false;

	delta[Y_AXIS]= 0 - detector_info.detect_travel;
	THEROBOT->delta_move(delta, detector_info.detect_rate, Y_AXIS + 1);
	// wait for it
	THECONVEYOR->wait_for_idle();
	if(THEKERNEL->is_halted()) return false;

	delta[Y_AXIS]= detector_info.detect_travel / 2;
	THEROBOT->delta_move(delta, detector_info.detect_rate, Y_AXIS + 1);
	// wait for it
	THECONVEYOR->wait_for_idle();
	if(THEKERNEL->is_halted()) return false;


	detecting = false;
	// switch off detector
	switch_state = false;
    ok = PublicData::set_value(switch_checksum, detector_switch_checksum, state_checksum, &switch_state);
    if (!ok) {
        THEKERNEL->streams->printf("ERROR: Failed switch off detector switch.\r\n");
        return false;
    }

    // reset position
    THEROBOT->reset_position_from_current_actuator_position();

    return detector_info.triggered;
}

bool ATCHandler::probe_detect() {
    // First wait for the queue to be empty
    THECONVEYOR->wait_for_idle();

    // get probe and calibrate states
    uint32_t probe_time;
    bool ok = PublicData::get_value(zprobe_checksum, get_zprobe_time_checksum, 0, &probe_time);
    if (ok) {
    	if (us_ticker_read() - probe_time < 5 * 1000 * 1000) {
    		return true;
    	}
    }

    return false;
}

void ATCHandler::home_clamp()
{
	THEKERNEL->streams->printf("Homing atc...\n");
    // First wait for the queue to be empty
    THECONVEYOR->wait_for_idle();

    atc_home_info.triggered = false;
    atc_home_info.clamp_status = UNHOMED;
    debounce = 0;
    atc_homing = true;

    // home atc
	float delta[ATC_AXIS + 1];
	for (size_t i = 0; i <= ATC_AXIS; i++) delta[i] = 0;
	delta[ATC_AXIS]= atc_home_info.max_travel; // we go the max
	THEROBOT->delta_move(delta, atc_home_info.homing_rate, ATC_AXIS + 1);
	// wait for it
	THECONVEYOR->wait_for_idle();
	if(THEKERNEL->is_halted()) return;

	atc_homing = false;

    if (!atc_home_info.triggered) {
        THEKERNEL->call_event(ON_HALT, nullptr);
        THEKERNEL->set_halt_reason(ATC_HOME_FAIL);
        THEKERNEL->streams->printf("ERROR: Homing atc failed - check the atc max travel settings\n");
        return;
    } else {
    	THEROBOT->reset_position_from_current_actuator_position();
    }

    // Move back
	for (size_t i = 0; i <= ATC_AXIS; i++) delta[i] = 0;
	delta[ATC_AXIS] = -atc_home_info.retract; // we go to retract position
	THEROBOT->delta_move(delta, atc_home_info.homing_rate, ATC_AXIS + 1);
	// wait for it
	THECONVEYOR->wait_for_idle();
	if(THEKERNEL->is_halted()) return;

	atc_home_info.clamp_status = CLAMPED;
	THEKERNEL->streams->printf("ATC homed!\r\n");

}

void ATCHandler::clamp_tool()
{
	if (atc_home_info.clamp_status == CLAMPED) {
		THEKERNEL->streams->printf("Already clamped!\n");
		return;
	}
	if (atc_home_info.clamp_status == UNHOMED) {
		home_clamp();
		return;
	}

    // First wait for the queue to be empty
    THECONVEYOR->wait_for_idle();

	float delta[ATC_AXIS + 1];
	for (size_t i = 0; i <= ATC_AXIS; i++) delta[i] = 0;
	delta[4] = atc_home_info.action_dist;
	THEROBOT->delta_move(delta, atc_home_info.homing_rate, ATC_AXIS + 1);
	// wait for it
	THECONVEYOR->wait_for_idle();
	if(THEKERNEL->is_halted()) return;

	// change clamp status
	atc_home_info.clamp_status = CLAMPED;
	THEKERNEL->streams->printf("ATC clamped!\r\n");
}

void ATCHandler::loose_tool()
{
	if (atc_home_info.clamp_status == LOOSED) {
		THEKERNEL->streams->printf("Already loosed!\n");
		return;
	}
	if (atc_home_info.clamp_status == UNHOMED) {
		home_clamp();
	}

	// First wait for the queue to be empty
    THECONVEYOR->wait_for_idle();

	float delta[ATC_AXIS + 1];
	for (size_t i = 0; i <= ATC_AXIS; i++) delta[i] = 0;
	delta[4] = -atc_home_info.action_dist;
	THEROBOT->delta_move(delta, atc_home_info.action_rate, ATC_AXIS + 1);
	// wait for it
	THECONVEYOR->wait_for_idle();
	if(THEKERNEL->is_halted()) return;

	// change clamp status
	atc_home_info.clamp_status = LOOSED;
	THEKERNEL->streams->printf("ATC loosed!\r\n");
}

void ATCHandler::set_tool_offset()
{
    float px, py, pz;
    uint8_t ps;
    std::tie(px, py, pz, ps) = THEROBOT->get_last_probe_position();
    if (ps == 1) {
        cur_tool_mz = pz;
        if (ref_tool_mz < 0) {
        	tool_offset = cur_tool_mz - ref_tool_mz;
        	const float offset[3] = {0.0, 0.0, tool_offset};
        	THEROBOT->saveToolOffset(offset, cur_tool_mz);
        }
    }

}

void ATCHandler::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode*>(argument);

    if (gcode->has_m) {
    	// gcode->stream->printf("Has m: %d\r\n", gcode->m);
    	if (gcode->m == 6 && gcode->has_letter('T')) {
    		// gcode->stream->printf("Has t\r\n");
    		if (atc_status != NONE) {
    			gcode->stream->printf("ATC already begun\r\n");
    			return;
    		}

    		THECONVEYOR->wait_for_idle();

    	    struct spindle_status ss;
    	    if (PublicData::get_value(pwm_spindle_control_checksum, get_spindle_status_checksum, &ss)) {
    	    	if (ss.state) {
    	    		PublicData::set_value(pwm_spindle_control_checksum, turn_off_spindle_checksum, nullptr);
    	    	}
    	    }

    	    if (PublicData::get_value(pwm_spindle_control_checksum, get_spindle_status_checksum, &ss)) {
    	    	if (ss.state) {
    	    		// Stop
    	    		THEKERNEL->streams->printf("Error: can not do ATC while spindle is running.\n");
			        THEKERNEL->set_halt_reason(ATC_HOME_FAIL);
			        THEKERNEL->call_event(ON_HALT, nullptr);
			        return;
    	    	}
    	    }

            int new_tool = gcode->get_value('T');
            if (new_tool > this->tool_number) {
		        THEKERNEL->call_event(ON_HALT, nullptr);
		        THEKERNEL->set_halt_reason(ATC_TOOL_INVALID);
            	gcode->stream->printf("ALARM: Invalid tool: T%d\r\n", new_tool);
            } else {
            	if (new_tool != active_tool) {
            		if (new_tool > -1 && THEKERNEL->get_laser_mode()) {
            			THEKERNEL->streams->printf("ALARM: Can not do ATC in laser mode!\n");
            			return;
            		}
                    // push old state
                    THEROBOT->push_state();
                    THEROBOT->get_axis_position(last_pos, 3);
                    set_inner_playing(true);
                    this->clear_script_queue();
                	if (this->active_tool < 0) {
                		gcode->stream->printf("Start picking new tool: T%d\r\n", new_tool);
                		// just pick up tool
                		atc_status = PICK;
                		this->fill_pick_scripts(new_tool, true);
                		this->fill_cali_scripts(new_tool == 0, false);
                	} else if (new_tool < 0) {
                		gcode->stream->printf("Start dropping current tool: T%d\r\n", this->active_tool);
                		// just drop tool
                		atc_status = DROP;
                		this->fill_drop_scripts(active_tool);
                		if (THEKERNEL->get_laser_mode()) {
                			this->fill_cali_scripts(false, false);
                		}
                	} else {
                		gcode->stream->printf("Start atc, old tool: T%d, new tool: T%d\r\n", this->active_tool, new_tool);
                		// full atc progress
                		atc_status = FULL;
                	    this->fill_drop_scripts(active_tool);
                	    this->fill_pick_scripts(new_tool, false);
                	    this->fill_cali_scripts(new_tool == 0, false);
                	}
            	} else if (new_tool == -1  && THEKERNEL->get_laser_mode()) {
            		// calibrate
                    THEROBOT->push_state();
                    THEROBOT->get_axis_position(last_pos, 3);
                    set_inner_playing(true);
                    this->clear_script_queue();
            		atc_status = CALI;
            		this->fill_cali_scripts(false, true);
            	}
            }
		} else if (gcode->m == 490)  {
			if (gcode->subcode == 0) {
				// home tool change
				home_clamp();
			} else if (gcode->subcode == 1) {
				// clamp tool
				clamp_tool();
			} else if (gcode->subcode == 2) {
				// loose tool
				loose_tool();
			}
		} else if (gcode->m == 491) {
			if (gcode->subcode == 1) {
				char buff[100];
				float tolerance = 0.1;
				if (gcode->has_letter('H')) {
		    		tolerance = gcode->get_value('H');
					if (tolerance < 0.02) {
						THEKERNEL->streams->printf("ERROR: Tool Break Check - tolerance set too small\n");
						THEKERNEL->call_event(ON_HALT, nullptr);
        				THEKERNEL->set_halt_reason(CALIBRATE_FAIL);
						return;
					}

				}
				//store current TLO
				float tlo = THEKERNEL->eeprom_data->TLO;

				// do calibrate to find new TLO
				THEROBOT->push_state();
				THEROBOT->get_axis_position(last_pos, 3);
				set_inner_playing(true);
				this->clear_script_queue();
				
				atc_status = CALI;
				this->fill_cali_scripts(active_tool == 0, true);

				THECONVEYOR->wait_for_idle();
				// lift z to safe position with fast speed
				snprintf(buff, sizeof(buff), "G53 G0 Z%.3f", THEROBOT->from_millimeters(this->safe_z_mm));
				this->script_queue.push(buff);
				snprintf(buff, sizeof(buff), "M491.2 H%.3f , P%.3f", tolerance, tlo);
				this->script_queue.push(buff);
				
				


			}else if (gcode->subcode == 2){
				float tlo = 0;
				float tolerance = 0.1;
				if (gcode->has_letter('H')) {
		    		tolerance = gcode->get_value('H');
					if (tolerance < 0.02) {
						THEKERNEL->streams->printf("ERROR: Tool Break Check - tolerance set too small\n");
						THEKERNEL->call_event(ON_HALT, nullptr);
        				THEKERNEL->set_halt_reason(CALIBRATE_FAIL);
						return;
					}

				}
				if (gcode->has_letter('P')) {
		    		tlo = gcode->get_value('P');
					if (tlo == 0) {
						THEKERNEL->streams->printf("No previous TLO included, aborting\n");
						return;
					}

				}
				float new_tlo = THEKERNEL->eeprom_data->TLO;
				THEKERNEL->streams->printf("Old: %.3f , new: %.3f\n",tlo,new_tlo);
				//test for breakage
				if (fabs(tlo - new_tlo) > tolerance) {
					THEKERNEL->streams->printf("ERROR: Tool Break Check - check tool for breakage\n");
					THEKERNEL->call_event(ON_HALT, nullptr);
					THEKERNEL->set_halt_reason(CALIBRATE_FAIL);
					return;
				}

			} else {
				// do calibrate
				THEROBOT->push_state();
				THEROBOT->get_axis_position(last_pos, 3);
				set_inner_playing(true);
				this->clear_script_queue();
				atc_status = CALI;
				this->fill_cali_scripts(active_tool == 0, true);

			}
		} else if (gcode->m == 492) {
			if (gcode->subcode == 0 || gcode->subcode == 1) {
				// check true
				if (!laser_detect()) {
			        THEKERNEL->call_event(ON_HALT, nullptr);
			        THEKERNEL->set_halt_reason(ATC_NO_TOOL);
			        THEKERNEL->streams->printf("ERROR: Tool confliction occured, please check tool rack!\n");
				}
			} else if (gcode->subcode == 2) {
				// check false
				if (laser_detect()) {
			        THEKERNEL->call_event(ON_HALT, nullptr);
			        THEKERNEL->set_halt_reason(ATC_HAS_TOOL);
			        THEKERNEL->streams->printf("ERROR: Tool confliction occured, please check tool rack!\n");
				}
			} else if (gcode->subcode == 3) {
				// check if the probe was triggered
				if (!probe_detect()) {
			        THEKERNEL->call_event(ON_HALT, nullptr);
			        THEKERNEL->set_halt_reason(PROBE_INVALID);
			        THEKERNEL->streams->printf("ERROR: Wireless probe dead or not set, please charge or set first!\n");
				}
			}
		} else if (gcode->m == 493) {
			if (gcode->subcode == 0 || gcode->subcode == 1) {
				// set tooll offset
				set_tool_offset();
			} else if (gcode->subcode == 2) {
				// set new tool
				if (gcode->has_letter('T')) {
		    		this->active_tool = gcode->get_value('T');
		    		// save current tool data to eeprom
		    		if (THEKERNEL->eeprom_data->TOOL != this->active_tool) {
		        	    THEKERNEL->eeprom_data->TOOL = this->active_tool;
		        	    THEKERNEL->write_eeprom_data();
		    		}

				} else {
					THEKERNEL->call_event(ON_HALT, nullptr);
					THEKERNEL->set_halt_reason(ATC_NO_TOOL);
					THEKERNEL->streams->printf("ERROR: No tool was set!\n");

				}
			}
		} else if (gcode->m == 494) {
			// control probe laser
			if (gcode->subcode == 0 || gcode->subcode == 1) {
				// open probe laser
				this->probe_laser_last = 0;
			} else if (gcode->subcode == 2) {
				// close probe laser
				this->probe_laser_last = 9999;
			}
		} else if (gcode->m == 495) {
			if (gcode->subcode == 3) {
				float tool_dia = 3.175;
				float probe_height = 9.0;
				if (gcode->has_letter('D')) {
					tool_dia = gcode->get_value('D');
				}
				if (gcode->has_letter('H')) {
					probe_height = gcode->get_value('H');
				}
	            THEROBOT->push_state();
				set_inner_playing(true);
				atc_status = AUTOMATION;
	            this->clear_script_queue();
				this->fill_xyzprobe_scripts(tool_dia, probe_height);
			} else {
				// Do Margin, ZProbe, Auto Leveling based on parameters, change probe tool if needed
				if (gcode->has_letter('X') && gcode->has_letter('Y')) {
	        		if (THEKERNEL->get_laser_mode()) {
	        			THEKERNEL->streams->printf("ALARM: Can not do Automatic work in laser mode!\n");
	        			return;
	        		}

					bool margin = false;
					bool zprobe = false;
					bool zprobe_abs = false;
					bool leveling = false;

					float x_path_pos = gcode->get_value('X');
					float y_path_pos = gcode->get_value('Y');

		    		float x_level_size = 0;
		    		float y_level_size = 0;
		    		int x_level_grids = 0;
		    		int y_level_grids = 0;
		    		float z_level_height = 5;
		    		float x_margin_pos_max = 0;
		    		float y_margin_pos_max = 0;
		    		float x_zprobe_offset = 0;
		    		float y_zprobe_offset = 0;
		    		if (gcode->has_letter('C') && gcode->has_letter('D')) {
		    			margin = true;
		    			x_margin_pos_max =  gcode->get_value('C');
		    			y_margin_pos_max =  gcode->get_value('D');
		    		}
		    		if (gcode->has_letter('O')) {
		    			zprobe = true;
		    			x_zprobe_offset =  gcode->get_value('O');
		    			if (gcode->has_letter('F')) {
			    			y_zprobe_offset =  gcode->get_value('F');
		    			} else {
			    			zprobe_abs = true;
		    			}
		    		}
		    		if (gcode->has_letter('A') && gcode->has_letter('B') && gcode->has_letter('I') && gcode->has_letter('J') && gcode->has_letter('H')) {
		    			leveling = true;
		    			x_level_size =  gcode->get_value('A');
		    			y_level_size =  gcode->get_value('B');
			    		x_level_grids = gcode->get_value('I');
			    		y_level_grids = gcode->get_value('J');
			    		z_level_height = gcode->get_value('H');
					}
		    		if (margin || zprobe || leveling) {
			            THEROBOT->push_state();
						set_inner_playing(true);
						atc_status = AUTOMATION;
			            this->clear_script_queue();
			            if (active_tool != 0) {
			            	// need to change to probe tool first
			        		gcode->stream->printf("Change to probe tool first!\r\n");
			                // save current position
			                THEROBOT->get_axis_position(last_pos, 3);
			        		if (active_tool > 0) {
			        			// drop current tool
			            		int old_tool = active_tool;
			            		// change to probe tool
			            		this->fill_drop_scripts(old_tool);
			        		}
		            		this->fill_pick_scripts(0, active_tool <= 0);
		            		this->fill_cali_scripts(true, false);
			            }
			            if (margin) {
			            	gcode->stream->printf("Auto scan margin\r\n");
			            	this->fill_margin_scripts(x_path_pos, y_path_pos, x_margin_pos_max, y_margin_pos_max);
			            }
			            if (zprobe) {
			            	if (zprobe_abs) {
				            	gcode->stream->printf("Auto z probe for 4 axis\r\n");
				            	this->fill_zprobe_abs_scripts();
			            	} else {
				            	gcode->stream->printf("Auto z probe, offset: %1.3f, %1.3f\r\n", x_zprobe_offset, y_zprobe_offset);
				            	this->fill_zprobe_scripts(x_path_pos, y_path_pos, x_zprobe_offset, y_zprobe_offset);
			            	}
			            }
			            if (leveling) {
			            	gcode->stream->printf("Auto leveling, grid: %d * %d height: %1.2f\r\n", x_level_grids, y_level_grids, z_level_height);
		            		this->fill_autolevel_scripts(x_path_pos, y_path_pos, x_level_size, y_level_size, x_level_grids, y_level_grids, z_level_height);
			            }
			            if (gcode->has_letter('P')) {
			            	gcode->stream->printf("Goto path origin first\r\n");
			            	this->fill_goto_origin_scripts(x_path_pos, y_path_pos);
			            }
		    		} else {
		    			if (gcode->has_letter('P')) {
							set_inner_playing(true);
							atc_status = AUTOMATION;
				            this->clear_script_queue();
		    				// goto path origin first
			            	gcode->stream->printf("Goto path origin first\r\n");
			            	this->fill_goto_origin_scripts(x_path_pos, y_path_pos);
		    			}
		    		}
				} else {
					gcode->stream->printf("ALARM: Miss Automation Parameter: X/Y\r\n");
				}
			}
		} else if (gcode->m == 496) {
			goto_position = gcode->subcode;
			// goto designative work position
			if (gcode->has_letter('X') && gcode->has_letter('Y')) {
				position_x = gcode->get_value('X');
				position_y = gcode->get_value('Y');
			}

		} else if (gcode->m == 497) {
		    // wait for the queue to be empty
		    THECONVEYOR->wait_for_idle();
			THEKERNEL->set_atc_state(gcode->subcode);
		} else if (gcode->m == 498) {
			if (gcode->subcode == 0 || gcode->subcode == 1) {
				THEKERNEL->streams->printf("EEPRROM Data: TOOL:%d\n", THEKERNEL->eeprom_data->TOOL);
				THEKERNEL->streams->printf("EEPRROM Data: TLO:%1.3f\n", THEKERNEL->eeprom_data->TLO);
				THEKERNEL->streams->printf("EEPRROM Data: TOOLMZ:%1.3f\n", THEKERNEL->eeprom_data->TOOLMZ);
				THEKERNEL->streams->printf("EEPRROM Data: REFMZ:%1.3f\n", THEKERNEL->eeprom_data->REFMZ);
				THEKERNEL->streams->printf("EEPRROM Data: G54: %1.3f, %1.3f, %1.3f\n", THEKERNEL->eeprom_data->G54[0], THEKERNEL->eeprom_data->G54[1], THEKERNEL->eeprom_data->G54[2]);
			} else if (gcode->subcode == 2) {
				// Show EEPROM DATA
				THEKERNEL->erase_eeprom_data();
			}
		} else if ( gcode->m == 499 ) {
			if (gcode->subcode == 0 || gcode->subcode == 1) {
				THEKERNEL->streams->printf("tool:%d ref:%1.3f cur:%1.3f offset:%1.3f\n", active_tool, ref_tool_mz, cur_tool_mz, tool_offset);
			} else if (gcode->subcode == 2) {
				THEKERNEL->streams->printf("probe -- mx:%1.1f my:%1.1f mz:%1.1f\n", probe_mx_mm, probe_my_mm, probe_mz_mm);
				for (int i = 0; i <=  tool_number; i ++) {
					THEKERNEL->streams->printf("tool%d -- mx:%1.1f my:%1.1f mz:%1.1f\n", atc_tools[i].num, atc_tools[i].mx_mm, atc_tools[i].my_mm, atc_tools[i].mz_mm);
				}
			}
		}
    } else if (gcode->has_g && gcode->g == 28 && gcode->subcode == 0) {
    	g28_triggered = true;
    }
}

void ATCHandler::on_main_loop(void *argument)
{
    if (this->atc_status != NONE) {
        if (THEKERNEL->is_halted()) {
            THEKERNEL->streams->printf("Kernel is halted!....\r\n");
            return;
        }

        if (THEKERNEL->is_suspending() || THEKERNEL->is_waiting()) {
        	return;
        }

        void *return_value;
        bool ok = PublicData::get_value( player_checksum, is_playing_checksum, &return_value );
        if (ok) {
            bool playing = *static_cast<bool *>(return_value);
            if (this->playing_file && !playing) {
            	this->clear_script_queue();

				this->atc_status = NONE;
				set_inner_playing(false);
				THEKERNEL->set_atc_state(ATC_NONE);

				// pop old state
				THEROBOT->pop_state();

				// if we were printing from an M command from pronterface we need to send this back
				THEKERNEL->streams->printf("Abort from ATC\n");

				return;
            }
        }

        while (!this->script_queue.empty()) {
        	THEKERNEL->streams->printf("%s\r\n", this->script_queue.front().c_str());
			struct SerialMessage message;
			message.message = this->script_queue.front();
			message.stream = THEKERNEL->streams;
			message.line = 0;
			this->script_queue.pop();

			// waits for the queue to have enough room
			THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
            return;
        }

		if (this->atc_status != AUTOMATION) {
	        // return to z clearance position
	        rapid_move(true, NAN, NAN, this->clearance_z);

	        // return to saved x and y position
	        rapid_move(true, last_pos[0], last_pos[1], NAN);
		}

        this->atc_status = NONE;

		set_inner_playing(false);

		THEKERNEL->set_atc_state(ATC_NONE);

        // pop old state
        THEROBOT->pop_state();

		// if we were printing from an M command from pronterface we need to send this back
		THEKERNEL->streams->printf("Done ATC\r\n");
    } else if (g28_triggered) {
		THEKERNEL->streams->printf("G28 means goto clearance position on CARVERA\n");
		THEROBOT->push_state();
		// goto z clearance
		rapid_move(true, NAN, NAN, this->clearance_z);
		// goto x and y clearance
		rapid_move(true, this->clearance_x, this->clearance_y, NAN);
		THECONVEYOR->wait_for_idle();
		THEROBOT->pop_state();
		g28_triggered = false;
    } else if (goto_position > -1) {
        rapid_move(true, NAN, NAN, this->clearance_z);
		if (goto_position == 0 || goto_position == 1) {
			// goto clearance
	        rapid_move(true, this->clearance_x, this->clearance_y, NAN);
		} else if (goto_position == 2) {
			// goto work origin
			// shrink A value first before move
			// shrink B value first before move
			rapid_move(false, 0, 0, NAN);
		} else if (goto_position == 3) {
			// goto anchor 1
			rapid_move(true, this->anchor1_x, this->anchor1_y, NAN);
		} else if (goto_position == 4) {
			// goto anchor 2
			rapid_move(true, this->anchor1_x + this->anchor2_offset_x, this->anchor1_y + this->anchor2_offset_y, NAN);
		} else if (goto_position == 5) {
			// goto designative work position
			if (position_x < 8888 && position_y < 8888) {
				rapid_move(false, position_x, position_y, NAN);
			}
		} else if (goto_position == 6) {
			// goto designative machine position
			if (position_x < 8888 && position_y < 8888) {
				rapid_move(true, position_x, position_y, NAN);
			}
		}
		position_x = 8888;
		position_y = 8888;
		goto_position = -1;
    }
}

// issue a coordinated move directly to robot, and return when done
// Only move the coordinates that are passed in as not nan
// NOTE must use G53 to force move in machine coordinates and ignore any WCS offsets
void ATCHandler::rapid_move(bool mc, float x, float y, float z)
{
    #define CMDLEN 128
    char *cmd= new char[CMDLEN]; // use heap here to reduce stack usage

    if (mc)
    	strcpy(cmd, "G53 G0 "); // G53 forces movement in machine coordinate system
    else
    	strcpy(cmd, "G90 G0 "); // G90 forces movement in machine coordinate system

    if(!isnan(x)) {
        size_t n= strlen(cmd);
        snprintf(&cmd[n], CMDLEN-n, " X%1.3f", THEROBOT->from_millimeters(x));
    }
    if(!isnan(y)) {
        size_t n= strlen(cmd);
        snprintf(&cmd[n], CMDLEN-n, " Y%1.3f", THEROBOT->from_millimeters(y));
    }
    if(!isnan(z)) {
        size_t n= strlen(cmd);
        snprintf(&cmd[n], CMDLEN-n, " Z%1.3f", THEROBOT->from_millimeters(z));
    }

    // send as a command line as may have multiple G codes in it
    struct SerialMessage message;
    message.message = cmd;
    delete [] cmd;

    message.stream = &(StreamOutput::NullStream);
    message.line = 0;
    THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );
    THEKERNEL->conveyor->wait_for_idle();

}

void ATCHandler::on_get_public_data(void* argument)
{
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);

    if(!pdr->starts_with(atc_handler_checksum)) return;

    if(pdr->second_element_is(get_tool_status_checksum)) {
    	if (this->active_tool >= 0) {
            struct tool_status *t= static_cast<tool_status*>(pdr->get_data_ptr());
            t->active_tool = this->active_tool;
            t->ref_tool_mz = this->ref_tool_mz;
            t->cur_tool_mz = this->cur_tool_mz;
            t->tool_offset = this->tool_offset;
            pdr->set_taken();
    	}
    } else if (pdr->second_element_is(get_atc_pin_status_checksum)) {
        char *data = static_cast<char *>(pdr->get_data_ptr());
        // cover endstop
        data[0] = (char)this->atc_home_info.pin.get();
        data[1] = (char)this->detector_info.detect_pin.get();
        pdr->set_taken();
    }
}

void ATCHandler::on_set_public_data(void* argument)
{
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);

    if(!pdr->starts_with(atc_handler_checksum)) return;

    if(pdr->second_element_is(set_ref_tool_mz_checksum)) {
        this->ref_tool_mz = cur_tool_mz;
        // update eeprom data if needed
        if (this->ref_tool_mz != THEKERNEL->eeprom_data->REFMZ) {
        	THEKERNEL->eeprom_data->REFMZ = this->ref_tool_mz;
		    THEKERNEL->write_eeprom_data();
        }
        this->tool_offset = 0.0;
        pdr->set_taken();
    }
}

bool ATCHandler::get_inner_playing() const
{
    void *returned_data;

    bool ok = PublicData::get_value( player_checksum, inner_playing_checksum, &returned_data );
    if (ok) {
        bool b = *static_cast<bool *>(returned_data);
        return b;
    }
    return false;
}

void ATCHandler::set_inner_playing(bool inner_playing)
{
	this->playing_file = PublicData::set_value( player_checksum, inner_playing_checksum, &inner_playing );
}
