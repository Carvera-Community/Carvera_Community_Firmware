/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "ATCHandler.h"

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "libs/gpio.h"
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
#define anchor_width_checksum		CHECKSUM("anchor_width")
#define anchor1_x_checksum			CHECKSUM("anchor1_x")
#define anchor1_y_checksum			CHECKSUM("anchor1_y")
#define anchor2_offset_x_checksum	CHECKSUM("anchor2_offset_x")
#define anchor2_offset_y_checksum	CHECKSUM("anchor2_offset_y")
#define rotation_offset_x_checksum	CHECKSUM("rotation_offset_x")
#define rotation_offset_y_checksum	CHECKSUM("rotation_offset_y")
#define rotation_offset_z_checksum	CHECKSUM("rotation_offset_z")
#define rotation_width_checksum		CHECKSUM("rotation_width")
#define toolrack_offset_x_checksum	CHECKSUM("toolrack_offset_x")
#define toolrack_offset_y_checksum	CHECKSUM("toolrack_offset_y")
#define toolrack_z_checksum			CHECKSUM("toolrack_z")
#define clearance_x_checksum		CHECKSUM("clearance_x")
#define clearance_y_checksum		CHECKSUM("clearance_y")
#define clearance_z_checksum		CHECKSUM("clearance_z")
#define skip_path_origin_checksum	CHECKSUM("skip_path_origin")
#define probe_mcs_x_checksum		CHECKSUM("probe_mcs_x")
#define probe_mcs_y_checksum		CHECKSUM("probe_mcs_y")
#define probe_mcs_z_checksum		CHECKSUM("probe_mcs_z")

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
    if(THEKERNEL->factory_set->FuncSetting & (1<<3))	//for CE1 expand
	{
    	tool_number = 8;
    }
    else
    {
    	tool_number = 6;
    }
	max_manual_tool_number = 1000000;
    g28_triggered = false;
    goto_position = -1;
    position_x = 8888;
    position_y = 8888;
    position_a = 88888888;
    position_b = 88888888;
    
    // Initialize one-off probe offsets
    probe_oneoff_x = 0.0;
    probe_oneoff_y = 0.0;
    probe_oneoff_z = 0.0;
    probe_oneoff_configured = false;
}

void ATCHandler::clear_script_queue(){
	while (!this->script_queue.empty()) {
		this->script_queue.pop();
	}
}

void ATCHandler::load_custom_tool_slots() {
    // Clear existing tool slots
    this->atc_tools.clear();

    // Read from custom_tool_slots.txt file on SD card
    string filename = "/sd/custom_tool_slots.txt";
    
    // printf("DEBUG: Loading custom tool slots from %s...\n", filename.c_str());
    // Check if file exists first
    if (!file_exists(filename)) {
        // File doesn't exist, no custom tool slots to load
        // printf("DEBUG: Custom tool slots file not found: %s\n", filename.c_str());
        return;
    }
    
    FILE *fp = fopen(filename.c_str(), "r");
    if (fp == NULL) {
        printf("WARNING: Failed to open %s for reading\n", filename.c_str());
        return;
    }

    // Parse the file line by line
    // Format: tool_num x y [z]
    // Example: 10 -3.526 -61.643 -119.1
    //          (Z is optional, uses previous tool's Z if not provided)
    int max_tool_num = -1;
    float last_valid_z = NAN;  // Track the last valid Z value to use for subsequent tools
    
    char buf[132];
    while (fgets(buf, sizeof(buf)-1, fp) != NULL) {
        string line(buf);
        
        // Skip comments and blank lines
        size_t begin = line.find_first_not_of(" \t");
        if (begin == string::npos || line[begin] == '#') {
            continue;
        }
        
        // Parse: tool_num x y [z]
        // Find tool number (first number)
        size_t end_tool = line.find_first_of(" \t", begin);
        if (end_tool == string::npos) {
            continue;
        }
        string tool_num_str = line.substr(begin, end_tool - begin);
        int tool_num = atoi(tool_num_str.c_str());
        
        if (tool_num < 0 || tool_num > 255) {
            continue;  // Invalid tool number (uint8_t range: 0-255)
        }
        
        // Find X coordinate
        size_t begin_x = line.find_first_not_of(" \t", end_tool);
        if (begin_x == string::npos || line[begin_x] == '#') {
            continue;
        }
        size_t end_x = line.find_first_of(" \t\r\n#", begin_x);
        if (end_x == string::npos) {
            continue;
        }
        string x_str = line.substr(begin_x, end_x - begin_x);
        float x_mm = strtof(x_str.c_str(), NULL);
        
        // Find Y coordinate
        size_t begin_y = line.find_first_not_of(" \t", end_x);
        if (begin_y == string::npos || line[begin_y] == '#') {
            continue;
        }
        size_t end_y = line.find_first_of(" \t\r\n#", begin_y);
        if (end_y == string::npos) {
            continue;
        }
        string y_str = line.substr(begin_y, end_y - begin_y);
        float y_mm = strtof(y_str.c_str(), NULL);
        
        // Find Z coordinate (optional)
        size_t begin_z = line.find_first_not_of(" \t", end_y);
        float z_mm = NAN;
        if (begin_z != string::npos && line[begin_z] != '#' && line[begin_z] != '\r' && line[begin_z] != '\n') {
            size_t end_z = line.find_first_of(" \t\r\n#", begin_z);
            size_t z_len = (end_z == string::npos) ? line.length() - begin_z : end_z - begin_z;
            string z_str = line.substr(begin_z, z_len);
            z_mm = strtof(z_str.c_str(), NULL);
        }
        
        // If Z is not provided, use the last valid Z value
        if (isnan(z_mm)) {
            if (!isnan(last_valid_z)) {
                z_mm = last_valid_z;  // Use previous tool's Z value
            } else {
                // First tool without Z - skip this tool (or could use a default)
                // For now, skip it to maintain safety
                continue;
            }
        } else {
            // Z is provided, update last_valid_z for future tools
            last_valid_z = z_mm;
        }
        
        // Track maximum tool number
        if (tool_num > max_tool_num) {
            max_tool_num = tool_num;
        }
        
        // Ensure vector is large enough
        if (tool_num >= (int)atc_tools.size()) {
            atc_tools.resize(tool_num + 1);
        }
        
        // Store tool slot immediately (no temporary maps needed)
        atc_tools[tool_num].num = tool_num;
        atc_tools[tool_num].set_mx_mm(x_mm);
        atc_tools[tool_num].set_my_mm(y_mm);
        atc_tools[tool_num].set_mz_mm(z_mm);
    }
    fclose(fp);
}

bool ATCHandler::is_custom_tool_defined(int tool_num) {
    if (tool_num < 0 || tool_num >= (int)atc_tools.size()) {
        return false;
    }
    return atc_tools[tool_num].is_valid();
}

void ATCHandler::add_custom_tool_slot(int tool_num, float x_mm, float y_mm, float z_mm) {
    // Validate inputs
    if (tool_num < 0 || tool_num > 255) {
        THEKERNEL->streams->printf("ERROR: Tool number must be between 0 and 255\n");
        return;
    }
    
    if (isnan(x_mm) || isnan(y_mm)) {
        THEKERNEL->streams->printf("ERROR: X and Y coordinates must be valid numbers\n");
        return;
    }
    
    // If Z is not provided (NAN), find the previous valid tool slot's Z value
    if (isnan(z_mm)) {
        // Search backwards from the current tool number to find the last valid Z
        float last_valid_z = NAN;
        for (int i = tool_num - 1; i >= 0; i--) {
            if (i < (int)atc_tools.size() && atc_tools[i].is_valid()) {
                last_valid_z = atc_tools[i].get_mz_mm();
                break;
            }
        }
        
        if (isnan(last_valid_z)) {
            THEKERNEL->streams->printf("ERROR: Z coordinate is required for the first tool slot (no previous Z value found)\n");
            return;
        }
        
        z_mm = last_valid_z;
        THEKERNEL->streams->printf("Using Z from previous tool slot: %.3f\n", z_mm);
    }
    
    // Ensure vector is large enough
    if (tool_num >= (int)atc_tools.size()) {
        atc_tools.resize(tool_num + 1);
    }
    
    // Update or add tool slot directly in atc_tools
    bool is_update = atc_tools[tool_num].is_valid();
    atc_tools[tool_num].num = tool_num;
    atc_tools[tool_num].set_mx_mm(x_mm);
    atc_tools[tool_num].set_my_mm(y_mm);
    atc_tools[tool_num].set_mz_mm(z_mm);
    
    if (is_update) {
        THEKERNEL->streams->printf("Updated tool slot %d: X=%.3f Y=%.3f Z=%.3f\n", tool_num, x_mm, y_mm, z_mm);
    } else {
        THEKERNEL->streams->printf("Added tool slot %d: X=%.3f Y=%.3f Z=%.3f\n", tool_num, x_mm, y_mm, z_mm);
    }
    
    // Count valid tool slots
    int valid_count = 0;
    for (const auto& tool : atc_tools) {
        if (tool.is_valid()) {
            valid_count++;
        }
    }
    THEKERNEL->streams->printf("Total custom tool slots: %d\n", valid_count);
}

void ATCHandler::remove_custom_tool_slot(int tool_num) {
    // Validate input
    if (tool_num < 0 || tool_num > 255) {
        THEKERNEL->streams->printf("ERROR: Tool number must be between 0 and 255\n");
        return;
    }
    
    // Check if tool exists
    if (tool_num >= (int)atc_tools.size() || !atc_tools[tool_num].is_valid()) {
        THEKERNEL->streams->printf("Tool slot %d is not defined\n", tool_num);
        return;
    }
    
    // Since the vector is indexed by tool number, we can't remove from the middle
    // without breaking indexing. Instead, we'll mark it as invalid
    
    // Mark tool as invalid by setting internal coordinates directly to -1
    // (Don't use set_mx_mm() etc. as they convert -1.0mm to -100, not the invalid marker -1)
    atc_tools[tool_num].mx_mm = -1;
    atc_tools[tool_num].my_mm = -1;
    atc_tools[tool_num].mz_mm = -1;
    
    THEKERNEL->streams->printf("Removed tool slot %d\n", tool_num);
}

void ATCHandler::save_custom_tool_slots_to_file() {
    // Write to SD card
    string filename = "/sd/custom_tool_slots.txt";
    FILE *fp = fopen(filename.c_str(), "w");
    
    if (fp == NULL) {
        THEKERNEL->streams->printf("ERROR: Failed to open %s for writing\n", filename.c_str());
        return;
    }
    
    // Write header comment
    fprintf(fp, "# Custom tool slot configuration\n");
    fprintf(fp, "# Format: tool_num x y [z]\n");
    fprintf(fp, "# Example: 10 -3.526 -61.643 -119.1\n");
    fprintf(fp, "#          (Z is optional, uses previous tool's Z if not provided)\n");
    fprintf(fp, "# This file is automatically generated. Do not edit manually.\n\n");
    
    // Write all valid tool slots (one line per tool)
    bool wrote_any = false;
    for (const auto& tool : atc_tools) {
        if (tool.is_valid()) {
            fprintf(fp, "%d %.3f %.3f %.3f\n", tool.num, tool.get_mx_mm(), tool.get_my_mm(), tool.get_mz_mm());
            wrote_any = true;
        }
    }
    
    fclose(fp);
    
    if (wrote_any) {
        THEKERNEL->streams->printf("Saved custom tool slot config to %s\n", filename.c_str());
    } else {
        THEKERNEL->streams->printf("No custom tool slot config to save\n");
    }
}


void ATCHandler::fill_calibrate_probe_anchor_scripts(bool invert_probe){
	THEKERNEL->streams->printf("Calibrating Probe Tip With Anchor 2\n");
	char buff[100];
	if (!THEROBOT->is_homed_all_axes()){
		return;
	}

	//print status
	snprintf(buff, sizeof(buff), ";Confirm that 3 axis probe is in collet\nand anchor 2 is installed\n");
	this->script_queue.push(buff);

	//move to clearance
	snprintf(buff, sizeof(buff), "G90 G53 G0 Z%.3f", THEROBOT->from_millimeters(this->clearance_z));
	this->script_queue.push(buff);

	//move to anchor 2 probe position
	snprintf(buff, sizeof(buff), "G91 G53 G0 X%.3f Y%.3f", this->anchor1_x + this->anchor2_offset_x - this->anchor_width/2, this->anchor1_y + this->anchor2_offset_y + 15);
	this->script_queue.push(buff);
	
	//probe -z
	if (!invert_probe){
		snprintf(buff, sizeof(buff), "G38.3 Z-100 F450");
		this->script_queue.push(buff);
	} else{
		snprintf(buff, sizeof(buff), "G38.5 Z-100 F450");
		this->script_queue.push(buff);
	}
	snprintf(buff, sizeof(buff), "G91 G54 G0 Z3");
	this->script_queue.push(buff);
	
	//execute calibration with specific values
	
	snprintf(buff, sizeof(buff), "M460.2 X%.3f E5 L2 I%i", this->anchor_width, invert_probe ? 1:0);
	this->script_queue.push(buff);

	
}

void ATCHandler::calibrate_set_value(Gcode *gcode)
{
	if (!gcode->has_letter('P')){
		THEKERNEL->streams->printf("Not enough variables given to M469\n Abort\n");
		return;
	} else{
		float final_x = 0;
		float final_y = 0;
		float final_z = 0;
		switch ((int)gcode->get_value('P')){
			case 0:
				//home off pin
				break;
			case 1:
				//calibrate anchor 1
				if (!gcode->has_letter('X') && !gcode->has_letter('Y')){
					THEKERNEL->streams->printf("Not enough variables given to M469\n Abort\n");
					return;
				}
				THEKERNEL->streams->printf("Previous Anchor 1 X: %.3f\nPrevious Anchor 1 Y: %.3f\n", gcode->get_value('X'), gcode->get_value('Y'));
				final_x = THEKERNEL->probe_outputs[3];
				final_y = THEKERNEL->probe_outputs[4];
				THEKERNEL->streams->printf("New Anchor 1 X: %.3f\nNew Anchor 1 Y: %.3f\n", final_x, final_y);
				this->anchor1_x = final_x;
				this->anchor1_y = final_y;
				THEKERNEL->streams->printf("These values have been temporarily set. You can test the position with M496.3\nTo make them permanent run:\nconfig-set sd coordinate.anchor1_x %.3f\nconfig-set sd coordinate.anchor1_y %.3f\n", final_x, final_y);
				break;
				break;
			case 2:
				//calibrate anchor 2
				if (!gcode->has_letter('X') && !gcode->has_letter('Y')){
					THEKERNEL->streams->printf("Not enough variables given to M469\n Abort\n");
					return;
				}
				THEKERNEL->streams->printf("Previous Anchor 2 X: %.3f\nPrevious Anchor 2 Y: %.3f\n", gcode->get_value('X'), gcode->get_value('Y'));
				final_x = THEKERNEL->probe_outputs[3] - this->anchor1_x;
				final_y = THEKERNEL->probe_outputs[4] - this->anchor1_y;
				THEKERNEL->streams->printf("New Anchor 2 X: %.3f\nNew Anchor 2 Y: %.3f\n", final_x, final_y);
				this->anchor2_offset_x = final_x;
				this->anchor2_offset_y = final_y;
				THEKERNEL->streams->printf("These values have been temporarily set. You can test the position with M496.4\nTo make them permanent run:\nconfig-set sd coordinate.anchor2_x %.3f\nconfig-set sd coordinate.anchor2_y %.3f\n you will need to check calibration on all other machine positions\n", final_x, final_y);
				break;
			case 3:
				//calibrate atc positions - unable to complete due to lack of x axis travel
			case 4:
				//calibrate 4th axis headstock
				if (!gcode->has_letter('Y')){
					THEKERNEL->streams->printf("Not enough variables given to M469\n Abort\n");
					return;
				}
				THEKERNEL->streams->printf("Previous 4th Headstock Y: %.3f\n", gcode->get_value('Y'));
				final_y = THEKERNEL->probe_outputs[4] - this->anchor1_y;
				THEKERNEL->streams->printf("New 4th Headstock Y: %.3f\n", final_y);
				this->rotation_offset_y = final_y;
				THEKERNEL->streams->printf("These values have been temporarily set. \nTo make them permanent run:\nconfig-set sd coordinate.rotation_offset_y %.3f\n", final_y);
				break;
			case 5:
				//caibrate 4th axis height
				//M469.6 A#120 B#119 C#118 Z%.3f P5
				if (!gcode->has_letter('A') && !gcode->has_letter('B') && !gcode->has_letter('C') && !gcode->has_letter('D') && !gcode->has_letter('E') && !gcode->has_letter('Z') && !gcode->has_letter('R')){
					THEKERNEL->streams->printf("Not enough variables given to M469\n Abort\n");
					return;
				}
				THEKERNEL->streams->printf("Previous 4th Height: %.3f\n", gcode->get_value('Z'));

				
				final_y = -(gcode->get_value('B') + gcode->get_value('C') + gcode->get_value('D') + gcode->get_value('E'))/4 + gcode->get_value('A') + gcode->get_value('R')/2;
				THEKERNEL->streams->printf("New 4th Height: %.3f\n", final_y);
				//this->rotation_offset_z = final_y;
				THEKERNEL->streams->printf("These values have been temporarily set. \nTo make them permanent run:\nconfig-set sd coordinate.rotation_offset_z %.3f\n", final_y);
				break;
			default:
				return;
				break;
		}
	}
}

void ATCHandler::calibrate_anchor1(Gcode *gcode) //M469.1
{
	THEKERNEL->streams->printf("Calibrating Anchor 1\n");
	char buff[100];
	if (!THEROBOT->is_homed_all_axes()){
		return;
	}

	bool invert_probe = false;

	if (gcode->has_letter('I')){
		if (gcode->get_value('I')){
			invert_probe = true;
		}
	}

	//print status
	snprintf(buff, sizeof(buff), ";Confirm that a 3 axis probe is in collet\nand anchor 1 is installed and clear\n");
	this->script_queue.push(buff);


	//move to clearance
	snprintf(buff, sizeof(buff), "G90 G53 G0 Z%.3f", THEROBOT->from_millimeters(this->clearance_z));
	this->script_queue.push(buff);

	//move to anchor 2 probe position
	snprintf(buff, sizeof(buff), "G91 G53 G0 X%.3f Y%.3f", this->anchor1_x - 5, this->anchor1_y - 5);
	this->script_queue.push(buff);
	
	//probe -z
	if (!invert_probe){
		snprintf(buff, sizeof(buff), "G38.3 Z-105 F450");
		this->script_queue.push(buff);
	} else{
		snprintf(buff, sizeof(buff), "G38.5 Z-105 F450");
		this->script_queue.push(buff);
	}
	snprintf(buff, sizeof(buff), "G91 G54 G0 Z3");
	this->script_queue.push(buff);

	snprintf(buff, sizeof(buff), "G91 G54 G0 X15 Y15");
	this->script_queue.push(buff);
	
	//execute calibration with specific values
	
	snprintf(buff, sizeof(buff), "M463 X-20 Y-20 H6 C1 I%i", invert_probe ? 1:0);
	this->script_queue.push(buff);

	snprintf(buff, sizeof(buff), "M469.6 X%.3f Y%.3f P1", this->anchor1_x , this->anchor1_y);
	this->script_queue.push(buff);
}

void ATCHandler::calibrate_anchor2(Gcode *gcode)//M469.2
{
	THEKERNEL->streams->printf("Calibrating Anchor 2\n");
	char buff[100];
	if (!THEROBOT->is_homed_all_axes()){
		return;
	}

	bool invert_probe = false;

	if (gcode->has_letter('I')){
		if (gcode->get_value('I')){
			invert_probe = true;
		}
	}

	//print status
	snprintf(buff, sizeof(buff), ";Confirm that a 3 axis probe is in collet\nand anchor 2 is installed and clear\n");
	this->script_queue.push(buff);

	//move to clearance
	snprintf(buff, sizeof(buff), "G90 G53 G0 Z%.3f", THEROBOT->from_millimeters(this->clearance_z));
	this->script_queue.push(buff);

	//move to anchor 2 probe position
	snprintf(buff, sizeof(buff), "G91 G53 G0 X%.3f Y%.3f", this->anchor1_x + this->anchor2_offset_x - 7, this->anchor1_y + this->anchor2_offset_y - 7);
	this->script_queue.push(buff);
	
	//probe -z
	if (!invert_probe){
		snprintf(buff, sizeof(buff), "G38.3 Z-105 F450");
		this->script_queue.push(buff);
	} else{
		snprintf(buff, sizeof(buff), "G38.5 Z-105 F450");
		this->script_queue.push(buff);
	}
	snprintf(buff, sizeof(buff), "G91 G54 G0 Z3");
	this->script_queue.push(buff);

	snprintf(buff, sizeof(buff), "G91 G54 G0 X20 Y20");
	this->script_queue.push(buff);
	
	//execute calibration with specific values
	
	snprintf(buff, sizeof(buff), "M463 X-20 Y-20 H6 C1 I%i", invert_probe ? 1:0);
	this->script_queue.push(buff);

	snprintf(buff, sizeof(buff), "M469.6 X%.3f Y%.3f P2", this->anchor2_offset_x , this->anchor2_offset_y);
	this->script_queue.push(buff);
}

void ATCHandler::calibrate_a_axis_headstock(Gcode *gcode)//M469.4
{
	float headstock_width = this->rotation_width/2 + 5;
	float probe_height= this->rotation_offset_z;

	THEKERNEL->streams->printf("Calibrating A Axis Headstock Center\n");
	char buff[100];
	if (!THEROBOT->is_homed_all_axes()){
		return;
	}

	bool invert_probe = false;

	if (gcode->has_letter('I')){
		if (gcode->get_value('I')){
			invert_probe = true;
		}
	}
	if (gcode->has_letter('Y')){
		headstock_width = gcode->get_value('Y')/2+5;
	}
	if (gcode->has_letter('H')){
		probe_height = gcode->get_value('H');
	}

	//print status
	snprintf(buff, sizeof(buff), ";Confirm that a 3 axis probe is in collet\nand the 4th axis is installed and clear\n");
	this->script_queue.push(buff);

	//move to clearance
	snprintf(buff, sizeof(buff), "G90 G53 G0 Z%.3f", THEROBOT->from_millimeters(this->clearance_z));
	this->script_queue.push(buff);

	this->fill_zprobe_abs_scripts();
	
	//execute calibration with specific values
	snprintf(buff, sizeof(buff), "M462 Y%.3f E%.3f I%i", headstock_width , probe_height, invert_probe ? 1:0);
	this->script_queue.push(buff);

	snprintf(buff, sizeof(buff), "M469.6 Y%.3f P4" , this->rotation_offset_y);
	this->script_queue.push(buff);
	
}

void ATCHandler::calibrate_a_axis_height(Gcode *gcode) //M469.5
{
	float retract_height = 4.0;  // Default retract height between probes
	float x_axis_offset = 60;
	float pin_diameter = 6;

	THEKERNEL->streams->printf("Calibrating A Axis Center Height\n");
	char buff[100];
	if (!THEROBOT->is_homed_all_axes()){
		return;
	}

	bool invert_probe = false;

	if (gcode->has_letter('I')){
		if (gcode->get_value('I')){
			invert_probe = true;
		}
	}
	if (gcode->has_letter('X')){
		x_axis_offset = gcode->get_value('X');
	}
	if (gcode->has_letter('C')){
		retract_height = gcode->get_value('C');
	}
	if (gcode->has_letter('R')){
		pin_diameter = gcode->get_value('R');
	}

	//print status
	snprintf(buff, sizeof(buff), ";Confirm that a 3 axis probe is in collet\nand the 4th axis is installed with a pin and clear\n");
	this->script_queue.push(buff);
	snprintf(buff, sizeof(buff), ";This code uses variables #116-120\n");
	this->script_queue.push(buff);

	// First Z probe at anchor position using fill_zprobe_abs_scripts
	this->fill_zprobe_abs_scripts();

	// Store location to variable
	snprintf(buff, sizeof(buff), "#120 = #5023");
	this->script_queue.push(buff);

	//move to clearance
	snprintf(buff, sizeof(buff), "G90 G53 G0 Z%.3f", THEROBOT->from_millimeters(this->clearance_z));
	this->script_queue.push(buff);

	snprintf(buff, sizeof(buff), "G91 G53 G0 X%.3f", this->anchor1_x + this->rotation_offset_x + x_axis_offset);
	this->script_queue.push(buff);
	snprintf(buff, sizeof(buff), "G91 G53 G0 Y%.3f", this->anchor1_y + this->rotation_offset_y);
	this->script_queue.push(buff);
	
	//probe -z
	if (!invert_probe){
		snprintf(buff, sizeof(buff), "G38.3 Z-105 F450");
		this->script_queue.push(buff);
		snprintf(buff, sizeof(buff), "G91 G0 Z%.3f", retract_height);
		this->script_queue.push(buff);
		snprintf(buff, sizeof(buff), "G38.3 Z%.3f F150", -(retract_height + 4.0));
		this->script_queue.push(buff);
		snprintf(buff, sizeof(buff), "#119 = #5023");
		this->script_queue.push(buff);
	} else{
		snprintf(buff, sizeof(buff), "G38.5 Z-105 F450");
		this->script_queue.push(buff);
		snprintf(buff, sizeof(buff), "G91 G0 Z%.3f", retract_height);
		this->script_queue.push(buff);
		snprintf(buff, sizeof(buff), "G38.5 Z%.3f F150", -(retract_height + 4.0));
		this->script_queue.push(buff);
		snprintf(buff, sizeof(buff), "#119 = #5023");
		this->script_queue.push(buff);
	}
	snprintf(buff, sizeof(buff), "G91 G0 Z%.3f", retract_height);
	this->script_queue.push(buff);
	snprintf(buff, sizeof(buff), "G91 G0 A90");
	this->script_queue.push(buff);
	
	if (!invert_probe){
		snprintf(buff, sizeof(buff), "G38.3 Z%.3f F150", -(retract_height + 4.0));
		this->script_queue.push(buff);
		snprintf(buff, sizeof(buff), "#118 = #5023");
		this->script_queue.push(buff);
	} else{
		snprintf(buff, sizeof(buff), "G38.5 Z%.3f F150", -(retract_height + 4.0));
		this->script_queue.push(buff);
		snprintf(buff, sizeof(buff), "#118 = #5023");
		this->script_queue.push(buff);
	}
	snprintf(buff, sizeof(buff), "G91 G0 Z%.3f", retract_height);
	this->script_queue.push(buff);
	snprintf(buff, sizeof(buff), "G91 G0 A90");
	this->script_queue.push(buff);
	
	if (!invert_probe){
		snprintf(buff, sizeof(buff), "G38.3 Z%.3f F150", -(retract_height + 4.0));
		this->script_queue.push(buff);
		snprintf(buff, sizeof(buff), "#117 = #5023");
		this->script_queue.push(buff);
	} else{
		snprintf(buff, sizeof(buff), "G38.5 Z%.3f F150", -(retract_height + 4.0));
		this->script_queue.push(buff);
		snprintf(buff, sizeof(buff), "#117 = #5023");
		this->script_queue.push(buff);
	}
	snprintf(buff, sizeof(buff), "G91 G0 Z%.3f", retract_height);
	this->script_queue.push(buff);
	snprintf(buff, sizeof(buff), "G91 G0 A90");
	this->script_queue.push(buff);
	
	if (!invert_probe){
		snprintf(buff, sizeof(buff), "G38.3 Z%.3f F150", -(retract_height + 4.0));
		this->script_queue.push(buff);
		snprintf(buff, sizeof(buff), "#116 = #5023");
		this->script_queue.push(buff);
	} else{
		snprintf(buff, sizeof(buff), "G38.5 Z%.3f F150", -(retract_height + 4.0));
		this->script_queue.push(buff);
		snprintf(buff, sizeof(buff), "#116 = #5023");
		this->script_queue.push(buff);
	}
	//move to clearance
	snprintf(buff, sizeof(buff), "G90 G53 G0 Z%.3f", THEROBOT->from_millimeters(this->clearance_z));
	this->script_queue.push(buff);



	
	//execute calibration with specific values
	snprintf(buff, sizeof(buff), "M469.6 A#120 B#119 C#118 D#117 E#116 R%.3f Z%.3f P5" , pin_diameter, this->rotation_offset_z);
	this->script_queue.push(buff);
}

void ATCHandler::home_machine_with_pin(Gcode *gcode)//M469
{
	//test is homed
    //move to clearance height
    //move to xy position for reference pin, value in config
    //probe -z to top of pin
    //run boss probing routine with values in config
    //compare center position to stored xy reference location in a way that the controller understands
}



void ATCHandler::fill_change_scripts(int new_tool, bool clear_z, int old_tool = NAN, bool wait_after_empty = false, float custom_TLO = NAN) {
	char buff[100];

	// move to tool change position
	snprintf(buff, sizeof(buff), "G53 G0 Z%.3f", THEROBOT->from_millimeters(this->clearance_z));
	this->script_queue.push(buff);

    // move x and y to active tool position
	snprintf(buff, sizeof(buff), "G53 G0 X%.3f Y%.3f", THEROBOT->from_millimeters(probe_mx_mm - 22.0), THEROBOT->from_millimeters(probe_my_mm));
	this->script_queue.push(buff);

	this->script_queue.push("M497.2");

	if (THEKERNEL->factory_set->FuncSetting & (1<<2)){
		if (!isnan(old_tool) && old_tool != -1){
			// Enter tool changing waiting status
			this->script_queue.push("M490.3");
		}
		//open
		this->script_queue.push("M490.2");
		
		if (new_tool != -1){
			snprintf(buff, sizeof(buff), "M493.5 T%d", new_tool);
			this->script_queue.push(buff);
			// Enter tool changing waiting status
			this->script_queue.push("M490.3");
			//close
			this->script_queue.push("M490.1");
			// set new tool
			snprintf(buff, sizeof(buff), "M493.2 T%d", new_tool);
			this->script_queue.push(buff);
			// Enter tool changing waiting status for calibration
			this->script_queue.push("M490.3");
		}else{
			// set new tool
			snprintf(buff, sizeof(buff), "M493.2 T%d", new_tool);
			this->script_queue.push(buff);
			if (wait_after_empty){
				// Enter tool changing waiting status for calibration
				this->script_queue.push("M490.3");
			}
		}

		if (!isnan(custom_TLO)) {
			//print status
			snprintf(buff, sizeof(buff), "Tool is now installed and TLO set as %.3f." , custom_TLO );
			this->script_queue.push(buff);
			//set tool length offset
			snprintf(buff, sizeof(buff), "M493.3 Z%.3f",custom_TLO);
			this->script_queue.push(buff);
		}
	}else{
		this->script_queue.push("M490.1");
		// set new tool
		snprintf(buff, sizeof(buff), "M493.2 T%d", new_tool);
		this->script_queue.push(buff);
	}
}

void ATCHandler::fill_manual_drop_scripts(int old_tool) {
	char buff[100];
	struct atc_tool *current_tool = &atc_tools[old_tool];
	// set atc status
	this->script_queue.push("M497.1");
	//make extra sure the spindle is off
	snprintf(buff, sizeof(buff), "M5");
	this->script_queue.push(buff);
	// lift z to safe position with fast speed
	snprintf(buff, sizeof(buff), "G90 G53 G0 Z%.3f", THEROBOT->from_millimeters(this->clearance_z));
	this->script_queue.push(buff);
	//move to clearance
	snprintf(buff, sizeof(buff), "G90 G53 G0 X%.3f Y%.3f", THEROBOT->from_millimeters(probe_mx_mm - 22.0), THEROBOT->from_millimeters(probe_my_mm));
	this->script_queue.push(buff);

	//print status
	snprintf(buff, sizeof(buff), ";Ready to drop tool %d. Prepare to catch tool, resume will loosen collet\n", old_tool);
	this->script_queue.push(buff);
	//pause program to allow inputs
	snprintf(buff, sizeof(buff), "M600.5");
	this->script_queue.push(buff);
	//Drop Tool After Resume
	snprintf(buff, sizeof(buff), "M490.2");
	this->script_queue.push(buff);
	// set new tool to -1
	this->script_queue.push("M493.2 T-1");
	//print status
	snprintf(buff, sizeof(buff), ";collet should now be empty and machine paused.\nremove hands from machine");
	this->script_queue.push(buff);
	snprintf(buff, sizeof(buff), ";program will resume on play\n");
	this->script_queue.push(buff);
	//pause program
	snprintf(buff, sizeof(buff), "M600.5");
	this->script_queue.push(buff);
}

void ATCHandler::fill_manual_pickup_scripts(int new_tool, bool clear_z, bool auto_calibrate = false, float custom_TLO = NAN) {
	char buff[100];
	struct atc_tool *current_tool = &atc_tools[new_tool];
	// set atc status
	this->script_queue.push("M497.2");
	// lift z to safe position with fast speed
	snprintf(buff, sizeof(buff), "G90 G53 G0 Z%.3f", THEROBOT->from_millimeters(this->clearance_z));
	this->script_queue.push(buff);
	//move to clearance
	snprintf(buff, sizeof(buff), "G90 G53 G0 X%.3f Y%.3f", THEROBOT->from_millimeters(probe_mx_mm - 22.0), THEROBOT->from_millimeters(probe_my_mm));
	this->script_queue.push(buff);
	
	// loose tool
	this->script_queue.push("M490.2");
	//print status
	snprintf(buff, sizeof(buff), ";Ready to install tool %d. Resume will tighten the collet\n", new_tool);
	this->script_queue.push(buff);
	//pause
	snprintf(buff, sizeof(buff), "M600.5");
	this->script_queue.push(buff);
	// clamp tool
	this->script_queue.push("M490.1");
	// set new tool
	snprintf(buff, sizeof(buff), "M493.2 T%d", new_tool);
	this->script_queue.push(buff);

	if (auto_calibrate){
		//print status
		snprintf(buff, sizeof(buff), ";Tool is now installed.\nRemove hands from the machine\nResume will auto calibrate and continue\n");
		this->script_queue.push(buff);
		//pause
		snprintf(buff, sizeof(buff), "M600.5");
		this->script_queue.push(buff);
		this->fill_cali_scripts(new_tool == 0 || new_tool >= 999990,false);
	}else if (!isnan(custom_TLO)) {
		//print status
		snprintf(buff, sizeof(buff), ";Tool is now installed and TLO set as %.3f.\n Resume will continue program\n" , custom_TLO );
		this->script_queue.push(buff);
		//set tool length offset
		snprintf(buff, sizeof(buff), "M493.3 Z%.3f",custom_TLO);
		this->script_queue.push(buff);
		//pause
		snprintf(buff, sizeof(buff), "M600.5");
		this->script_queue.push(buff);
	} else {
		//print status
		snprintf(buff, sizeof(buff), ";Tool installed but TLO not set. Manually set TLO, see M493.3 H#");
		this->script_queue.push(buff);
		snprintf(buff, sizeof(buff), ";Leave machine at a safe Z height when done\n");
		this->script_queue.push(buff);
		snprintf(buff, sizeof(buff), ";Resume will continue program\n");
		this->script_queue.push(buff);
		
		//pause
		snprintf(buff, sizeof(buff), "M600.5");
		this->script_queue.push(buff);
	}
}

void ATCHandler::fill_drop_scripts(int old_tool) {
	char buff[100];

	if (!THEROBOT->is_homed_all_axes()) {
		return;
	};
	
	struct atc_tool *current_tool = &atc_tools[old_tool];
	// set atc status
	this->script_queue.push("M497.1");
    // lift z axis to atc start position
	snprintf(buff, sizeof(buff), "G53 G0 Z%.3f", THEROBOT->from_millimeters(this->clearance_z));
	this->script_queue.push(buff);
    // move x and y to active tool position
	snprintf(buff, sizeof(buff), "G53 G0 X%.3f Y%.3f", THEROBOT->from_millimeters(current_tool->get_mx_mm()), THEROBOT->from_millimeters(current_tool->get_my_mm()));
	this->script_queue.push(buff);
	// move around to see if tool rack is empty
	this->script_queue.push("M492.2");
    // move x and y to reseted tool position
	snprintf(buff, sizeof(buff), "G53 G0 X%.3f Y%.3f", THEROBOT->from_millimeters(current_tool->get_mx_mm()), THEROBOT->from_millimeters(current_tool->get_my_mm()));
	this->script_queue.push(buff);
    // drop z axis to z position with fast speed
	snprintf(buff, sizeof(buff), "G53 G1 Z%.3f F%.3f", THEROBOT->from_millimeters(current_tool->get_mz_mm() + safe_z_offset_mm), THEROBOT->from_millimeters(fast_z_rate));
	this->script_queue.push(buff);
    // drop z axis with slow speed
	snprintf(buff, sizeof(buff), "G53 G1 Z%.3f F%.3f", THEROBOT->from_millimeters(current_tool->get_mz_mm()), THEROBOT->from_millimeters(slow_z_rate));
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
	if (!THEROBOT->is_homed_all_axes()) {
		return;
	};
	struct atc_tool *current_tool = &atc_tools[new_tool];
	// set atc status
	this->script_queue.push("M497.2");
	// lift z to safe position with fast speed
	snprintf(buff, sizeof(buff), "G53 G0 Z%.3f", THEROBOT->from_millimeters(clear_z ? this->clearance_z : this->safe_z_empty_mm));
	this->script_queue.push(buff);
	// move x and y to new tool position
	snprintf(buff, sizeof(buff), "G53 G0 X%.3f Y%.3f", THEROBOT->from_millimeters(current_tool->get_mx_mm()), THEROBOT->from_millimeters(current_tool->get_my_mm()));
	this->script_queue.push(buff);
	// move around to see if tool rack is filled
	this->script_queue.push("M492.1");
	// loose tool
	this->script_queue.push("M490.2");
	// move x and y to reseted tool position
	snprintf(buff, sizeof(buff), "G53 G0 X%.3f Y%.3f", THEROBOT->from_millimeters(current_tool->get_mx_mm()), THEROBOT->from_millimeters(current_tool->get_my_mm()));
	this->script_queue.push(buff);
    // drop z axis to z position with fast speed
	snprintf(buff, sizeof(buff), "G53 G1 Z%.3f F%.3f", THEROBOT->from_millimeters(current_tool->get_mz_mm() + safe_z_offset_mm), THEROBOT->from_millimeters(fast_z_rate));
	this->script_queue.push(buff);
    // drop z axis with slow speed
	snprintf(buff, sizeof(buff), "G53 G1 Z%.3f F%.3f", THEROBOT->from_millimeters(current_tool->get_mz_mm()), THEROBOT->from_millimeters(slow_z_rate));
	this->script_queue.push(buff);
	// clamp tool
	this->script_queue.push("M490.1");
	// lift z to safe position with fast speed
	if (CARVERA == THEKERNEL->factory_set->MachineModel){
		snprintf(buff, sizeof(buff), "G53 G0 Z%.3f", THEROBOT->from_millimeters(this->safe_z_mm));
	}else{
		snprintf(buff, sizeof(buff), "G53 G0 Z%.3f", THEROBOT->from_millimeters(this->clearance_z));
	}
	this->script_queue.push(buff);
	// move around to see if tool rack is empty, halt if not
	this->script_queue.push("M492.2");
	// set new tool
	snprintf(buff, sizeof(buff), "M493.2 T%d", new_tool);
	this->script_queue.push(buff);

}

void ATCHandler::fill_cali_scripts(bool is_probe, bool clear_z) {
	char buff[100];
	if (!THEROBOT->is_homed_all_axes()) {
		return;
	};

	if(is_probe){
	// open probe laser
		this->script_queue.push("M494.1");
	}
	// set atc status
	this->script_queue.push("M497.3");
	
	if(CARVERA == THEKERNEL->factory_set->MachineModel)
	{
		// clamp tool if in laser mode
		if (THEKERNEL->get_laser_mode()) {
			this->script_queue.push("M490.1");
		}
	}
	if (CARVERA_AIR == THEKERNEL->factory_set->MachineModel){
		clear_z = true;
	}
	
	// lift z to safe position with fast speed
	snprintf(buff, sizeof(buff), "G53 G0 Z%.3f", THEROBOT->from_millimeters(clear_z ? this->clearance_z : this->safe_z_mm));
	this->script_queue.push(buff);
	// move x and y to calibrate position
	// Use one-off offsets if configured, otherwise use standard probe position
	float probe_x = probe_mx_mm + (this->probe_oneoff_configured ? this->probe_oneoff_x : 0.0);
	float probe_y = probe_my_mm + (this->probe_oneoff_configured ? this->probe_oneoff_y : 0.0);
	snprintf(buff, sizeof(buff), "G53 G0 X%.3f Y%.3f", THEROBOT->from_millimeters(probe_x), THEROBOT->from_millimeters(probe_y));
	this->script_queue.push(buff);
	// do calibrate with fast speed
    if(CARVERA == THEKERNEL->factory_set->MachineModel)	//ATC 
    {
		// Use one-off Z offset if configured, otherwise use standard probe Z position
		float probe_z = probe_mz_mm + (this->probe_oneoff_configured ? this->probe_oneoff_z : 0.0);
		snprintf(buff, sizeof(buff), "G38.6 Z%.3f F%.3f", probe_z, probe_fast_rate);
	}
	else	//Manual Tool Change
	{
		// Use one-off Z offset if configured, otherwise use toolrack Z position
		float probe_z = toolrack_z - 10 + (this->probe_oneoff_configured ? this->probe_oneoff_z : 0.0);
		snprintf(buff, sizeof(buff), "G38.6 Z%.3f F%.3f", probe_z, probe_fast_rate);
	}
	this->script_queue.push(buff);
	// lift a bit
	snprintf(buff, sizeof(buff), "G91 G0 Z%.3f", THEROBOT->from_millimeters(probe_retract_mm));
	this->script_queue.push(buff);
	// do calibrate with slow speed
	// Use one-off Z offset if configured, otherwise use standard offset
	float slow_probe_z = -1 - probe_retract_mm + (this->probe_oneoff_configured ? this->probe_oneoff_z : 0.0);
	snprintf(buff, sizeof(buff), "G38.6 Z%.3f F%.3f", slow_probe_z, probe_slow_rate);
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
	
	if(is_probe){
		// close probe laser	
	    this->script_queue.push("M494.2");
	}
}

void ATCHandler::fill_margin_scripts(float x_pos, float y_pos, float x_pos_max, float y_pos_max) {
	char buff[100];

	// set atc status
	this->script_queue.push("M497.4");

    // open probe laser
	this->script_queue.push("M494.0");
	
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
    //this->script_queue.push("M494.2");

}

void ATCHandler::fill_goto_origin_scripts(float x_pos, float y_pos) {
	char buff[100];

	// lift z to clearance position with fast speed
	snprintf(buff, sizeof(buff), "G53 G0 Z%.3f", THEROBOT->from_millimeters(this->clearance_z));
	this->script_queue.push(buff);

	if(!this->skip_path_origin){
		// goto start position
		snprintf(buff, sizeof(buff), "G90 G0 X%.3f Y%.3f", THEROBOT->from_millimeters(x_pos), THEROBOT->from_millimeters(y_pos));
		this->script_queue.push(buff);
	}

}

void ATCHandler::fill_zprobe_scripts(float x_pos, float y_pos, float x_offset, float y_offset) {
	char buff[100];

	// set atc status
	this->script_queue.push("M497.5");
	
    // open wired probe laser
    this->script_queue.push("M494.1");

	// lift z to safe position with fast speed
	snprintf(buff, sizeof(buff), "G53 G0 Z%.3f", THEROBOT->from_millimeters(this->clearance_z));
	this->script_queue.push(buff);

	// goto z probe position
	snprintf(buff, sizeof(buff), "G90 G0 X%.3f Y%.3f", THEROBOT->from_millimeters(x_pos + x_offset), THEROBOT->from_millimeters(y_pos + y_offset));
	this->script_queue.push(buff);

	// do probe with fast speed
	if(THEKERNEL->factory_set->FuncSetting & (1<<2))	//ATC 
    {
		snprintf(buff, sizeof(buff), "G38.2 Z%.3f F%.3f", probe_mz_mm, probe_fast_rate);
	}
	else	//Manual Tool Change
	{
		snprintf(buff, sizeof(buff), "G38.2 Z%.3f F%.3f", toolrack_z, probe_fast_rate);
	}
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
	
    // close wired probe laser
    this->script_queue.push("M494.2");
}

void ATCHandler::fill_zprobe_abs_scripts() {
	char buff[100];
	if (!THEROBOT->is_homed_all_axes()) {
		return;
	};
	// set atc status
	this->script_queue.push("M497.5");	
	
    // open wired probe laser
    this->script_queue.push("M494.1");

	// lift z to safe position with fast speed
	snprintf(buff, sizeof(buff), "G53 G0 Z%.3f", THEROBOT->from_millimeters(clearance_z));
	this->script_queue.push(buff);

	// goto z probe position
	if(!(THEKERNEL->factory_set->FuncSetting & (1<<0)) )
    {
		snprintf(buff, sizeof(buff), "G53 G0 X%.3f Y%.3f", THEROBOT->from_millimeters(anchor1_x + rotation_offset_x - 3), THEROBOT->from_millimeters(anchor1_y + rotation_offset_y));
	}
	else
	{
		//snprintf(buff, sizeof(buff), "G53 G0 X%.3f Y%.3f", THEROBOT->from_millimeters(anchor1_x + rotation_offset_x - 7), THEROBOT->from_millimeters(anchor1_y + rotation_offset_y));
		// move y to clearance
		snprintf(buff, sizeof(buff), "G53 G0 Y%.3f", THEROBOT->from_millimeters(this->clearance_y));
		this->script_queue.push(buff);
		
		snprintf(buff, sizeof(buff), "G53 G0 X%.3f", THEROBOT->from_millimeters(anchor1_x + rotation_offset_x - 7) );
		this->script_queue.push(buff);
		
		snprintf(buff, sizeof(buff), "G53 G0 Y%.3f", THEROBOT->from_millimeters(anchor1_y + rotation_offset_y));
	}
	this->script_queue.push(buff);

	// do probe with fast speed	
	if(THEKERNEL->factory_set->FuncSetting & (1<<2))	//ATC 
    {
		snprintf(buff, sizeof(buff), "G38.2 Z%.3f F%.3f", probe_mz_mm, probe_fast_rate);
	}
	else	//Manual Tool Change
	{
		snprintf(buff, sizeof(buff), "G38.2 Z%.3f F%.3f", toolrack_z, probe_fast_rate);
	}
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
	
    // close wired probe laser
    this->script_queue.push("M494.2");
}

void ATCHandler::fill_xyzprobe_scripts(float tool_dia, float probe_height) {
	char buff[100];

	// set atc status
	this->script_queue.push("M497.5");

	// open wired probe laser
	this->script_queue.push("M494.1");

	// do z probe with slow speed
	if(THEKERNEL->factory_set->FuncSetting & (1<<2))	//ATC 
    {
		snprintf(buff, sizeof(buff), "G38.2 Z%.3f F%.3f", probe_mz_mm, probe_slow_rate);
	}
	else	//Manual Tool Change
	{
		snprintf(buff, sizeof(buff), "G38.2 Z%.3f F%.3f", toolrack_z, probe_slow_rate);
	}
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
	
	// close wired probe laser
	this->script_queue.push("M494.2");

}


void ATCHandler::fill_autolevel_scripts(float x_pos, float y_pos,
		float x_size, float y_size, int x_grids, int y_grids, float height)
{
	char buff[100];
	if (!THEROBOT->is_homed_all_axes()) {
		return;
	};
	// set atc status
	this->script_queue.push("M497.6");
	
	// open wired probe laser
    this->script_queue.push("M494.0");
	
	// goto x and y path origin
	snprintf(buff, sizeof(buff), "G90 G0 X%.3f Y%.3f", THEROBOT->from_millimeters(x_pos), THEROBOT->from_millimeters(y_pos));
	this->script_queue.push(buff);

	// do auto leveling
	snprintf(buff, sizeof(buff), "G32R1X0Y0A%.3fB%.3fI%dJ%dH%.3f", x_size, y_size, x_grids, y_grids, height);
	this->script_queue.push(buff);
	
	// close wired probe laser
    //this->script_queue.push("M494.2");
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
    
    if(CARVERA_AIR == THEKERNEL->factory_set->MachineModel)
	{
    	THEKERNEL->slow_ticker->attach(10, this, &ATCHandler::beep_beep);
    }

    // load data from eeprom
    this->active_tool = THEKERNEL->eeprom_data->TOOL;
	if(CARVERA == THEKERNEL->factory_set->MachineModel || CARVERA_AIR == THEKERNEL->factory_set->MachineModel){
		this->ref_tool_mz = -115.34; // Represents the machine Z coordinate when the tool length is 0
	}else{
		this->ref_tool_mz = -115.34; // In preparation for the Z1. Update this value when the Z1 is implemented
	}
    if (THEKERNEL->eeprom_data->REFMZ != this->ref_tool_mz)
    {
        THEKERNEL->eeprom_data->REFMZ = this->ref_tool_mz;
        THEKERNEL->write_eeprom_data();
    }
    this->cur_tool_mz = THEKERNEL->eeprom_data->TOOLMZ;
    this->tool_offset = THEKERNEL->eeprom_data->TLO;
	
	this->target_tool = -1;
	this->beep_state = BP_SLEEP;
	this->beep_count = 0;
	
}

void ATCHandler::on_config_reload(void *argument)
{
	char buff[10];
	
	if(CARVERA == THEKERNEL->factory_set->MachineModel)
	{
		atc_home_info.pin.from_string( THEKERNEL->config->value(atc_checksum, endstop_pin_checksum)->by_default("1.0^" )->as_string())->as_input();
	}
	else
	{
		atc_home_info.pin.from_string( THEKERNEL->config->value(atc_checksum, endstop_pin_checksum)->by_default("1.4^" )->as_string())->as_input();
	}
	atc_home_info.debounce_ms    = THEKERNEL->config->value(atc_checksum, debounce_ms_checksum)->by_default(1  )->as_number();
	atc_home_info.max_travel    = THEKERNEL->config->value(atc_checksum, max_travel_mm_checksum)->by_default(8  )->as_number();
	atc_home_info.retract    = THEKERNEL->config->value(atc_checksum, homing_retract_mm_checksum)->by_default(3  )->as_number();
	atc_home_info.action_dist    = THEKERNEL->config->value(atc_checksum, action_mm_checksum)->by_default(1  )->as_number();
	atc_home_info.homing_rate    = THEKERNEL->config->value(atc_checksum, homing_rate_mm_s_checksum)->by_default(0.4f )->as_number();
	atc_home_info.action_rate    = THEKERNEL->config->value(atc_checksum, action_rate_mm_s_checksum)->by_default(0.25f)->as_number();

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
	
	this->anchor_width = THEKERNEL->config->value(coordinate_checksum, anchor_width_checksum)->by_default(15  )->as_number();
	this->anchor1_x = THEKERNEL->config->value(coordinate_checksum, anchor1_x_checksum)->by_default(-359  )->as_number();
	this->anchor1_y = THEKERNEL->config->value(coordinate_checksum, anchor1_y_checksum)->by_default(-234  )->as_number();
	this->anchor2_offset_x = THEKERNEL->config->value(coordinate_checksum, anchor2_offset_x_checksum)->by_default(90  )->as_number();
	this->anchor2_offset_y = THEKERNEL->config->value(coordinate_checksum, anchor2_offset_y_checksum)->by_default(45.65F  )->as_number();
	
	if(CARVERA == THEKERNEL->factory_set->MachineModel)
	{
		this->toolrack_z = THEKERNEL->config->value(coordinate_checksum, toolrack_z_checksum)->by_default(-105  )->as_number();
		this->toolrack_offset_x = THEKERNEL->config->value(coordinate_checksum, toolrack_offset_x_checksum)->by_default(356  )->as_number();
		this->toolrack_offset_y = THEKERNEL->config->value(coordinate_checksum, toolrack_offset_y_checksum)->by_default(0  )->as_number();
		this->lowest_tool_number = 0;
	}
	else if(CARVERA_AIR == THEKERNEL->factory_set->MachineModel)
	{
		this->toolrack_z = THEKERNEL->config->value(coordinate_checksum, toolrack_z_checksum)->by_default(-108  )->as_number();
		this->toolrack_offset_x = THEKERNEL->config->value(coordinate_checksum, toolrack_offset_x_checksum)->by_default(126  )->as_number();
		this->toolrack_offset_y = THEKERNEL->config->value(coordinate_checksum, toolrack_offset_y_checksum)->by_default(196  )->as_number();
		this->lowest_tool_number = 1;
	}

	// Load configurable probe position (absolute MCS coordinates) before first use
	this->probe_mcs_x = THEKERNEL->config->value(coordinate_checksum, probe_mcs_x_checksum)->by_default(NAN)->as_number();
	this->probe_mcs_y = THEKERNEL->config->value(coordinate_checksum, probe_mcs_y_checksum)->by_default(NAN)->as_number();
	this->probe_mcs_z = THEKERNEL->config->value(coordinate_checksum, probe_mcs_z_checksum)->by_default(NAN)->as_number();
	this->probe_position_configured = !isnan(this->probe_mcs_x) || !isnan(this->probe_mcs_y) || !isnan(this->probe_mcs_z);
	
	// Load custom tool slots configuration
	// Delay loading to ensure system is fully initialized
	this->load_custom_tool_slots();

	// Calculate probe position - use configured absolute MCS coordinates if available, otherwise use hardcoded values
	if(CARVERA == THEKERNEL->factory_set->MachineModel){
			probe_mx_mm = this->anchor1_x + this->toolrack_offset_x;
			probe_my_mm = this->anchor1_y + this->toolrack_offset_y + 180;
			probe_mz_mm = this->toolrack_z - 40;
	}else{
			probe_mx_mm = anchor1_x + 280;
			probe_my_mm = anchor1_y + 196;
			probe_mz_mm = this->toolrack_z - 40;
	}

	
	if (atc_tools.empty()){
		// Use default tool slot configuration
		if(THEKERNEL->factory_set->FuncSetting & (1<<3))	//for CE1 expand
		{
			for (int i = 0; i <=  8; i ++) {
				struct atc_tool tool;
				tool.num = i;
			    // lift z axis to atc start position
				snprintf(buff, sizeof(buff), "tool%d", i);
				tool.set_mx_mm(this->anchor1_x + this->toolrack_offset_x);
				tool.set_my_mm(this->anchor1_y + this->toolrack_offset_y -5 + (i == 0 ? 219 : (8 - i) * 25));
				tool.set_mz_mm(this->toolrack_z - 4.5);
				atc_tools.push_back(tool);
			}
			// Calculate probe position - use configured absolute MCS coordinates if available, otherwise use hardcoded values

			probe_mx_mm = this->anchor1_x + this->toolrack_offset_x;
			probe_my_mm = this->anchor1_y + this->toolrack_offset_y -5 + 197;
			probe_mz_mm = this->toolrack_z - 44.5;
		}
		else
		{
			for (int i = 0; i <=  6; i ++) {
				struct atc_tool tool;
				tool.num = i;
			    // lift z axis to atc start position
				snprintf(buff, sizeof(buff), "tool%d", i);
				tool.set_mx_mm(this->anchor1_x + this->toolrack_offset_x);
				tool.set_my_mm(this->anchor1_y + this->toolrack_offset_y + (i == 0 ? 210 : (6 - i) * 30));
				tool.set_mz_mm(this->toolrack_z);
				atc_tools.push_back(tool);
			}
		}
	}

		// Use one-off offsets if configured, otherwise use standard manual position
		if (this->probe_position_configured) {
			probe_mx_mm = isnan(this->probe_mcs_x) ? (probe_mx_mm) : this->probe_mcs_x;
			probe_my_mm = isnan(this->probe_mcs_y) ? (probe_my_mm) : this->probe_mcs_y;
			probe_mz_mm = isnan(this->probe_mcs_z) ? (probe_mz_mm) : this->probe_mcs_z;
		}
	
	if(CARVERA == THEKERNEL->factory_set->MachineModel)
	{
		this->rotation_offset_x = THEKERNEL->config->value(coordinate_checksum, rotation_offset_x_checksum)->by_default(-8  )->as_number();
		this->rotation_offset_y = THEKERNEL->config->value(coordinate_checksum, rotation_offset_y_checksum)->by_default(37.5F  )->as_number();
		this->rotation_offset_z = THEKERNEL->config->value(coordinate_checksum, rotation_offset_z_checksum)->by_default(22.5F  )->as_number();
	
		this->clearance_x = THEKERNEL->config->value(coordinate_checksum, clearance_x_checksum)->by_default(-75  )->as_number();
		this->clearance_y = THEKERNEL->config->value(coordinate_checksum, clearance_y_checksum)->by_default(-3  )->as_number();
		this->clearance_z = THEKERNEL->config->value(coordinate_checksum, clearance_z_checksum)->by_default(-3  )->as_number();
	}
	else if(CARVERA_AIR == THEKERNEL->factory_set->MachineModel)
	{
		this->rotation_offset_x = THEKERNEL->config->value(coordinate_checksum, rotation_offset_x_checksum)->by_default(30.0F)->as_number();
		this->rotation_offset_y = THEKERNEL->config->value(coordinate_checksum, rotation_offset_y_checksum)->by_default(82.5F  )->as_number();
		this->rotation_offset_z = THEKERNEL->config->value(coordinate_checksum, rotation_offset_z_checksum)->by_default(23.0F  )->as_number();
	
		this->clearance_x = THEKERNEL->config->value(coordinate_checksum, clearance_x_checksum)->by_default(-5  )->as_number();
		this->clearance_y = THEKERNEL->config->value(coordinate_checksum, clearance_y_checksum)->by_default(-21  )->as_number();
		this->clearance_z = THEKERNEL->config->value(coordinate_checksum, clearance_z_checksum)->by_default(-5  )->as_number();
	}
	this->rotation_width = THEKERNEL->config->value(coordinate_checksum, rotation_width_checksum)->by_default(100 )->as_number();

	this->skip_path_origin = THEKERNEL->config->value(atc_checksum, skip_path_origin_checksum)->by_default(false)->as_bool();
}

void ATCHandler::on_halt(void* argument)
{
    uint8_t halt_reason;
    if (argument == nullptr ) {

        abort();

		if(CARVERA_AIR == THEKERNEL->factory_set->MachineModel)
	    {
    		halt_reason = THEKERNEL->get_halt_reason();
    		if (halt_reason <= 20) 
    		{
				this->beep_alarm();
			}
			else
			{
				this->beep_error();
			}
		}
	}
}

void ATCHandler::abort(){
	// Safety: If we were interrupted in the middle of calibration, mark tool as
	// not calibrated so that TLO measurement will be re-run on next tool use
	if (this->atc_status == CALI) {
		THEROBOT->set_tool_not_calibrated(true);
	}
	this->atc_status = NONE;
	this->clear_script_queue();
	this->set_inner_playing(false);
	THEKERNEL->set_atc_state(ATC_NONE);
	if(THEKERNEL->factory_set->FuncSetting & (1<<2))	//ATC 
	{
		this->atc_home_info.clamp_status = UNHOMED;
		THEKERNEL->set_tool_waiting(false);
	}else	//Manual Tool Change
	{
		THEKERNEL->set_tool_waiting(false);
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
	if(CARVERA == THEKERNEL->factory_set->FuncSetting)	//ATC 
	{
		if (this->probe_laser_last < 120) {
			this->probe_laser_last ++;
			PublicData::set_value(atc_handler_checksum, set_wp_laser_checksum, nullptr);
		}
	}
	else 	//Manual Tool Change
	{
		if (THEKERNEL->is_probeLaserOn()) 
		{
			if (this->probe_laser_last > 0) 
			{
				this->probe_laser_last --;
			}
			if (this->probe_laser_last == 0 || this->active_tool) {
	    		bool b = false;
	            PublicData::set_value( switch_checksum, detector_switch_checksum, state_checksum, &b );
	            THEKERNEL->set_probeLaser(false);
			}
		} 
		else
		{
			this->probe_laser_last = 300;
		}
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
        THEKERNEL->set_halt_reason(ATC_HOME_FAIL);
        THEKERNEL->call_event(ON_HALT, nullptr);
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
        if (ref_tool_mz < 1) {
        	tool_offset = cur_tool_mz - ref_tool_mz;
        	const float offset[3] = {0.0, 0.0, tool_offset};
        	THEROBOT->saveToolOffset(offset, cur_tool_mz);
		} else{
			THEKERNEL->eeprom_data->REFMZ = -10;
			THEKERNEL->write_eeprom_data();
			THEKERNEL->call_event(ON_HALT, nullptr);
			tool_offset = cur_tool_mz - ref_tool_mz;
			const float offset[3] = {0.0, 0.0, tool_offset};
			THEROBOT->saveToolOffset(offset, cur_tool_mz);
			THEKERNEL->set_halt_reason(MANUAL);
			THEKERNEL->streams->printf("ERROR: warning, unexpected reference tool length found, reset machine then recalibrate tool\n");
			return;
		}
    }

}

void ATCHandler::set_tlo_by_offset(float z_axis_offset){
	// new TLO = Current TLO - (current WCS - z_axis_offset)
	float mpos[3];
	Robot::wcs_t pos;
	THEROBOT->get_current_machine_position(mpos);
	// current_position/mpos includes the compensation transform so we need to get the inverse to get actual position
	if(THEROBOT->compensationTransform) THEROBOT->compensationTransform(mpos, true, false); // get inverse compensation transform
	pos = THEROBOT->mcs2wcs(mpos);
	cur_tool_mz = cur_tool_mz + THEROBOT->from_millimeters(std::get<Z_AXIS>(pos)) - z_axis_offset;

	if (ref_tool_mz < 1) {
		tool_offset = cur_tool_mz - ref_tool_mz;
		const float offset[3] = {0.0, 0.0, tool_offset};
		THEROBOT->saveToolOffset(offset, cur_tool_mz);
	}else{
		THEKERNEL->eeprom_data->REFMZ = -10;
		THEKERNEL->write_eeprom_data();
		THEKERNEL->call_event(ON_HALT, nullptr);
		tool_offset = cur_tool_mz - ref_tool_mz;
		const float offset[3] = {0.0, 0.0, tool_offset};
		THEROBOT->saveToolOffset(offset, cur_tool_mz);
		THEKERNEL->set_halt_reason(MANUAL);
		THEKERNEL->streams->printf("ERROR: warning, unexpected reference tool length found, reset machine then recalibrate tool\n");
		return;
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
    	    	if (ss.state) 
    	    	{
    	    		// Stop
    	    		
					if(THEKERNEL->factory_set->FuncSetting & (1<<2))	//ATC 
					{
	    	    		THEKERNEL->streams->printf("Error: can not do ATC while spindle is running.\n");
				        THEKERNEL->set_halt_reason(ATC_HOME_FAIL);
				    }
				    else	//Manual Tool Change
				    {
				    	THEKERNEL->streams->printf("Error: can not change tool while spindle is running.\n");
			        	THEKERNEL->set_halt_reason(CALIBRATE_FAIL);
				    }
			        THEKERNEL->call_event(ON_HALT, nullptr);
			        return;
    	    	}
    	    }

			// Handle one-off probe position offsets

			this->probe_oneoff_x = 0.0;
			this->probe_oneoff_y = 0.0;
			this->probe_oneoff_z = 0.0;
			this->probe_oneoff_configured = false;

			if (gcode->has_letter('X')) {
				this->probe_oneoff_x = gcode->get_value('X');
				this->probe_oneoff_configured = true;
			}
			if (gcode->has_letter('Y')) {
				this->probe_oneoff_y = gcode->get_value('Y');
				this->probe_oneoff_configured = true;
			}
			if (gcode->has_letter('Z')) {
				this->probe_oneoff_z = gcode->get_value('Z');
				this->probe_oneoff_configured = true;
			}

            int new_tool = gcode->get_value('T');
            if(THEKERNEL->factory_set->FuncSetting & (1<<2))	//ATC 
			{
	            if (new_tool > this->max_manual_tool_number){
					THEKERNEL->call_event(ON_HALT, nullptr);
					THEKERNEL->set_halt_reason(ATC_TOOL_INVALID);
					gcode->stream->printf("ALARM: Invalid tool: T%d\r\n", new_tool);
				} else if (new_tool >= 0 && !this->is_custom_tool_defined(new_tool) && new_tool <= this->tool_number && CARVERA == THEKERNEL->factory_set->MachineModel) {
					THEKERNEL->call_event(ON_HALT, nullptr);
					THEKERNEL->set_halt_reason(ATC_TOOL_INVALID);
					gcode->stream->printf("ALARM: Tool T%d not defined in tool slots\r\n", new_tool);
				} else if (new_tool != active_tool) {
					if (new_tool > -1 && THEKERNEL->get_laser_mode()) {
						THEKERNEL->streams->printf("ALARM: Can not do ATC in laser mode!\n");
						return;
					}
					
					// push old state
					bool auto_calibrate = true;
					float custom_TLO = NAN;
					THEROBOT->push_state();
					THEROBOT->get_axis_position(last_pos, 3);
					set_inner_playing(true);
					this->clear_script_queue();

					if (gcode->has_letter('H')){
						custom_TLO = gcode->get_value('H');
						auto_calibrate = false;
					}
					if (gcode->has_letter('C')){
						auto_calibrate = gcode->get_value('C');
					}
					//drop current tool
					if (this->active_tool >= this->lowest_tool_number && (this->is_custom_tool_defined(this->active_tool) || this->active_tool <= this->tool_number)){ //drop atc tool
						THEKERNEL->streams->printf("Start dropping current tool: T%d\r\n", this->active_tool);
						// just drop tool
						atc_status = DROP;
						this->fill_drop_scripts(active_tool);
					}else if((this->active_tool > this->tool_number || this->active_tool < this->lowest_tool_number) && !this->is_custom_tool_defined(this->active_tool) && this->active_tool != -1){ //drop manual tool
						THEKERNEL->streams->printf("Start manually dropping current tool: T%d\r\n", this->active_tool);
						atc_status = CHANGE;
						this->target_tool = -1;
						if (new_tool >= 0 && (this->is_custom_tool_defined(new_tool) || new_tool <= this->tool_number)){
							this->fill_change_scripts(-1, true, active_tool, true);
						}else{
							this->fill_change_scripts(-1, true, active_tool, false);
						}
					}
					
					//pick up new tool
					if((new_tool > this->tool_number || new_tool < this->lowest_tool_number) && !this->is_custom_tool_defined(new_tool) && new_tool != -1){
						THEKERNEL->streams->printf("Start manually picking new tool: T%d\r\n", new_tool);
						atc_status = CHANGE;
						this->fill_change_scripts(new_tool, true, -1, false, custom_TLO);
						
						if (auto_calibrate){
							this->fill_cali_scripts((new_tool == 0 || new_tool >= 999990), true); 
						}
					}else if(new_tool >= 0 && (this->is_custom_tool_defined(new_tool) || new_tool <= this->tool_number)){ //standard ATC
						THEKERNEL->streams->printf("Start picking new tool: T%d\r\n", new_tool);
						atc_status = PICK;
						this->fill_pick_scripts(new_tool, true);
						this->fill_cali_scripts(new_tool == 0 || new_tool >= 999990, false);
					} else if(new_tool == -1 && (THEKERNEL->get_laser_mode())) {
						THEROBOT->push_state();
						THEROBOT->get_axis_position(last_pos, 3);
						set_inner_playing(true);
						this->clear_script_queue();
						atc_status = CALI;
						this->fill_cali_scripts(false, true);
					}
				} else if (new_tool == active_tool && THEROBOT->get_tool_not_calibrated()) {
					// Tool is already selected but needs calibration (e.g., after e-stop during previous calibration)
					THEKERNEL->streams->printf("Tool T%d needs TLO calibration\r\n", new_tool);
					THEROBOT->push_state();
					THEROBOT->get_axis_position(last_pos, 3);
					set_inner_playing(true);
					this->clear_script_queue();
					atc_status = CALI;
					// Set TLO calibration flag to disable 3D probe crash detection
					bool tlo_calibrating = true;
					PublicData::set_value( zprobe_checksum, set_tlo_calibrating_checksum, &tlo_calibrating );
					this->fill_cali_scripts(new_tool == 0 || new_tool >= 999990, true);
				}
	        }
	        else	//Manual Tool Change for AIR
	        {
	        	if (new_tool != active_tool) 
	        	{
					// push old state
					THEROBOT->push_state();
					THEROBOT->get_axis_position(last_pos, 3);
					set_inner_playing(true);
					this->clear_script_queue();
					gcode->stream->printf("Please change the tool to: T%d\r\n", new_tool);
					// just pick up tool
					atc_status = CHANGE;
					this->target_tool = new_tool;
					this->fill_change_scripts(new_tool, true, active_tool);
					this->fill_cali_scripts((new_tool == 0 || new_tool >= 999990), true);
						} else if (new_tool == active_tool && THEROBOT->get_tool_not_calibrated()) {
							// Tool is already selected but needs calibration (e.g., after e-stop during previous calibration)
							THEKERNEL->streams->printf("Tool T%d needs TLO calibration\r\n", new_tool);
							THEROBOT->push_state();
							THEROBOT->get_axis_position(last_pos, 3);
							set_inner_playing(true);
							this->clear_script_queue();
							atc_status = CALI;
							// Set TLO calibration flag to disable 3D probe crash detection
							bool tlo_calibrating = true;
							PublicData::set_value( zprobe_checksum, set_tlo_calibrating_checksum, &tlo_calibrating );
							this->fill_cali_scripts((new_tool == 0 || new_tool >= 999990), true);
						}
	        }
		} else if (gcode->m == 460){
			
			if (gcode->subcode == 3) {
				bool invert_probe = false;
				if (gcode->has_letter('I')) {
					if (gcode->get_value('I') > 0 ){
						invert_probe = true;
					}
				}
				set_inner_playing(true);
				atc_status = AUTOMATION;
				this->clear_script_queue();
				this->fill_calibrate_probe_anchor_scripts(invert_probe);
			}
		} else if (gcode->m == 469) {
			if (gcode->subcode == 1){
				set_inner_playing(true);
				atc_status = AUTOMATION;
				this->clear_script_queue();
				calibrate_anchor1(gcode);
			}
			else if (gcode->subcode == 2){
				set_inner_playing(true);
				atc_status = AUTOMATION;
				this->clear_script_queue();
				calibrate_anchor2(gcode);
			} else if(gcode->subcode == 3){
				return;
			} else if(gcode->subcode == 4){
				set_inner_playing(true);
				atc_status = AUTOMATION;
				this->clear_script_queue();
				calibrate_a_axis_headstock(gcode);
			} else if(gcode->subcode ==5){
				set_inner_playing(true);
				atc_status = AUTOMATION;
				this->clear_script_queue();
				calibrate_a_axis_height(gcode);
			} else if(gcode->subcode == 6){
				set_inner_playing(true);
				atc_status = AUTOMATION;
				this->clear_script_queue();
				calibrate_set_value(gcode);
			} else {
				set_inner_playing(true);
				atc_status = AUTOMATION;
				this->clear_script_queue();
				home_machine_with_pin(gcode);
			}
		} else if (gcode->m == 490)  {
			if(THEKERNEL->factory_set->FuncSetting & (1<<2))	//ATC 
			{	            
				if (gcode->subcode == 0) 
				{
					// home tool change
					home_clamp();
				} 
				else if (gcode->subcode == 1) 
				{
					// clamp tool
					clamp_tool();
				} 
				else if (gcode->subcode == 2) 
				{
					// loose tool
					loose_tool();
				}else if (gcode->subcode == 3){
					THEKERNEL->set_tool_waiting(true);
				}else if (gcode->subcode == 4){
					THEKERNEL->set_tool_waiting(false);
				}
			}
			else	//Manual Tool Change
			{
				if (gcode->subcode == 0)
				{
					bool bgoback = false;
					GPIO stepin = GPIO(P1_22);
					GPIO dirpin = GPIO(P1_0);
					GPIO enpin = GPIO(P1_10);
					stepin.output();
					dirpin.output();
					enpin.output();
					GPIO alarmin = GPIO(P1_4);
					alarmin.input();
						
					gcode->stream->printf("check ATC motor beginning......\n");
					dirpin = 1;
					enpin = 0;
					
					for(unsigned int i=0;i<360;i++)
					{
						for(unsigned int j=0; j<889; j++)
						{
							stepin = 1;
							safe_delay_us(5);
							stepin = 0;
							safe_delay_us(5);
							if(alarmin.get())
							{
								bgoback = true;
								break;
							}
						}
						if(bgoback)
							break;
					}
					
					if(bgoback)
					{
						dirpin = 0;
						
						for(unsigned int i=0;i<90;i++)
						{
							for(unsigned int j=0; j<889; j++)
							{
								stepin = 1;
								safe_delay_us(5);
								stepin = 0;
								safe_delay_us(5);
							}
						}
					}
					
					enpin = 1;
					
					gcode->stream->printf("check ATC motor.\n");
				}
				else if (gcode->subcode == 1) 
				{
					// Enter tool change waiting status
					THEKERNEL->set_tool_waiting(true);
					this->beep_tool_change(this->target_tool);
				} 
				else if (gcode->subcode == 2) 
				{
					// Exit tool change waiting status
					THEKERNEL->set_tool_waiting(false);
				}
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
				
				//make sure spindle is off
				snprintf(buff, sizeof(buff), "M5");
				this->script_queue.push(buff);

				atc_status = CALI;
				// Set TLO calibration flag to disable 3D probe crash detection
				bool tlo_calibrating = true;
				PublicData::set_value( zprobe_checksum, set_tlo_calibrating_checksum, &tlo_calibrating );
				this->fill_cali_scripts(active_tool == 0 || active_tool >= 999990, true);

				THECONVEYOR->wait_for_idle();
				// lift z to safe position with fast speed
				
				snprintf(buff, sizeof(buff), "G53 G0 Z%.3f", THEROBOT->from_millimeters(this->safe_z_mm));
				this->script_queue.push(buff);
				snprintf(buff, sizeof(buff), "M491.2 H%.3f , P%.3f", tolerance, tlo);
				this->script_queue.push(buff);
				
				


			}else if (gcode->subcode == 2){
				float tlo = 0;
				float tolerance = 0.1;
				THECONVEYOR->wait_for_idle();
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
				
				// Handle one-off probe position offsets

				this->probe_oneoff_x = 0.0;
				this->probe_oneoff_y = 0.0;
				this->probe_oneoff_z = 0.0;
				this->probe_oneoff_configured = false;

				if (gcode->has_letter('X')) {
					this->probe_oneoff_x = gcode->get_value('X');
					this->probe_oneoff_configured = true;
				}
				if (gcode->has_letter('Y')) {
					this->probe_oneoff_y = gcode->get_value('Y');
					this->probe_oneoff_configured = true;
				}
				if (gcode->has_letter('Z')) {
					this->probe_oneoff_z = gcode->get_value('Z');
					this->probe_oneoff_configured = true;
				}

				// do calibrate
				THEROBOT->push_state();
				THEROBOT->get_axis_position(last_pos, 3);
				set_inner_playing(true);
				this->clear_script_queue();
				atc_status = CALI;
				// Set TLO calibration flag to disable 3D probe crash detection
				bool tlo_calibrating = true;
				PublicData::set_value( zprobe_checksum, set_tlo_calibrating_checksum, &tlo_calibrating );
				this->fill_cali_scripts(active_tool == 0 || active_tool >= 999990, true);

			}
		} else if (gcode->m == 492) {
			if(THEKERNEL->factory_set->FuncSetting & (1<<2))	//ATC 
			{
				if (gcode->subcode == 0 || gcode->subcode == 1) {
					// check true
					if (!laser_detect() && CARVERA == THEKERNEL->factory_set->MachineModel) {
				        THEKERNEL->set_halt_reason(ATC_NO_TOOL);
				        THEKERNEL->call_event(ON_HALT, nullptr);
				        THEKERNEL->streams->printf("ERROR: Unexpected tool absence detected, please check tool rack!\n");
					}
				} else if (gcode->subcode == 2) {
					// check false
					if (laser_detect()) {
				        THEKERNEL->set_halt_reason(ATC_HAS_TOOL);
				        THEKERNEL->call_event(ON_HALT, nullptr);
				        THEKERNEL->streams->printf("ERROR: Unexpected tool presence detected, please check tool rack!\n");
					}
				} else if (gcode->subcode == 3) {
					// check if the probe was triggered
					if (!probe_detect()) {
				        THEKERNEL->set_halt_reason(PROBE_INVALID);
				        THEKERNEL->call_event(ON_HALT, nullptr);
				        THEKERNEL->streams->printf("ERROR: Wireless probe dead or not set, please charge or set first!\n");
					}
				}
			}
			else	//Manual Tool Change
			{
				if (gcode->subcode == 3) 
				{
					// check if the probe was triggered
					if (!probe_detect()) {
				        THEKERNEL->set_halt_reason(PROBE_INVALID);
				        THEKERNEL->call_event(ON_HALT, nullptr);
				        THEKERNEL->streams->printf("ERROR: Probe dead or not set, please charge or set first!\n");
				}
			}
			}
		} else if (gcode->m == 493) {
			if (gcode->subcode == 0 || gcode->subcode == 1) {
				THEROBOT->set_tool_not_calibrated(false);
				set_tool_offset();
			} else if (gcode->subcode == 2) {
				// set new tool
				if (gcode->has_letter('T')) {
		    		this->active_tool = gcode->get_value('T');
					THEROBOT->set_tool_not_calibrated(true);
		    		// save current tool data to eeprom
		    		if (THEKERNEL->eeprom_data->TOOL != this->active_tool) {
		        	    THEKERNEL->eeprom_data->TOOL = this->active_tool;
		        	    THEKERNEL->write_eeprom_data();
		    		}
		    		
		    		// Clear one-off probe offsets when changing tools
		    		this->probe_oneoff_x = 0.0;
		    		this->probe_oneoff_y = 0.0;
		    		this->probe_oneoff_z = 0.0;
		    		this->probe_oneoff_configured = false;

				} else {
					THEKERNEL->set_halt_reason(ATC_NO_TOOL);
					THEKERNEL->call_event(ON_HALT, nullptr);
					THEKERNEL->streams->printf("ERROR: No tool was set!\n");

				}
			} else if (gcode->subcode == 3) { //set current tool offset
				
				
				if (gcode->has_letter('Z')) {
					cur_tool_mz = gcode->get_value('Z');
					if (ref_tool_mz < 1) {
						tool_offset = cur_tool_mz;// + ref_tool_mz;
						const float offset[3] = {0.0, 0.0, tool_offset};
						THEROBOT->saveToolOffset(offset, cur_tool_mz);
					}else{
						THEKERNEL->eeprom_data->REFMZ = -10;
						THEKERNEL->write_eeprom_data();
						THEKERNEL->call_event(ON_HALT, nullptr);
						tool_offset = cur_tool_mz - ref_tool_mz;
						const float offset[3] = {0.0, 0.0, tool_offset};
						THEROBOT->saveToolOffset(offset, cur_tool_mz);
						THEKERNEL->set_halt_reason(MANUAL);
						THEKERNEL->streams->printf("ERROR: warning, unexpected reference tool length found, reset machine then recalibrate tool\n");
						return;
					}
				}
				if (gcode->has_letter('H')) { //set tlo from current position. H is offset above Z0
					this->set_tlo_by_offset(gcode->get_value('H'));
					
				}


				THEKERNEL->streams->printf("current tool offset [%.3f] , reference tool offset [%.3f]\n",cur_tool_mz,ref_tool_mz);
			} else if (gcode->subcode == 4) { //report current TLO
				THEKERNEL->streams->printf("current tool offset [%.3f] , reference tool offset [%.3f]\n",cur_tool_mz,ref_tool_mz);
				if (this->probe_oneoff_configured) {
					THEKERNEL->streams->printf("one-off tool setter position offsets: X[%.3f] Y[%.3f] Z[%.3f]\n", this->probe_oneoff_x, this->probe_oneoff_y, this->probe_oneoff_z);
				} else {
					THEKERNEL->streams->printf("no one-off tool setter position offsets configured\n");
				}
				if (this->probe_position_configured) {
					THEKERNEL->streams->printf("Tool setter position (MCS): X[%.3f] Y[%.3f] Z[%.3f]\n", this->probe_mcs_x, this->probe_mcs_y, this->probe_mcs_z);
				} else {
					THEKERNEL->streams->printf("using default Tool setter position: X[%.3f] Y[%.3f] Z[%.3f]\n", probe_mx_mm, probe_my_mm, probe_mz_mm);
				}
			} else if (gcode->subcode == 5) {
				// set new tool
				if (gcode->has_letter('T')) {
		    		this->target_tool = gcode->get_value('T');
				}
			}
		} else if (gcode->m == 494) {
			if(CARVERA == THEKERNEL->factory_set->MachineModel)	//ATC 
			{
				// control probe laser
				if (gcode->subcode == 0 || gcode->subcode == 1) {
					// open probe laser
					this->probe_laser_last = 0;
				} else if (gcode->subcode == 2) {
					// close probe laser
					this->probe_laser_last = 9999;
				}
			}
			else	//Manual Tool Change
			{
				// control probe laser
				if (gcode->subcode == 0 ){
					// open probe laser
					bool b = true;
	            	PublicData::set_value( switch_checksum, detector_switch_checksum, state_checksum, &b );
	            	THEKERNEL->set_probeLaser(true);
				}
				else if( gcode->subcode == 1) {
					// open probe laser
					bool b = true;
	            	PublicData::set_value( switch_checksum, detector_switch_checksum, state_checksum, &b );
				} else if (gcode->subcode == 2) {
					// close probe laser
					bool b = false;
	            	PublicData::set_value( switch_checksum, detector_switch_checksum, state_checksum, &b );
				}

			}
		} else if (gcode->m == 495) {
			if (!THEROBOT->is_homed_all_axes()) {
				this->atc_status = NONE;
        		this->clear_script_queue();
        		this->set_inner_playing(false);
				return;
			};
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
					if ((this->active_tool == 0 || this->active_tool >= 999990) && THEROBOT->get_tool_not_calibrated()){
						THEKERNEL->streams->printf("ALARM: Probe not calibrated. Please calibrate probe before probing.\n");
						THEKERNEL->call_event(ON_HALT, nullptr);
						THEKERNEL->set_halt_reason(CALIBRATE_FAIL);
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
			            if (active_tool != 0 && active_tool < 999990) {
			            	// need to change to probe tool first
			        		gcode->stream->printf("Change to probe tool first!\r\n");
			                // save current position
			                THEROBOT->get_axis_position(last_pos, 3);
									                
							if(THEKERNEL->factory_set->FuncSetting & (1<<2))	//ATC 
							{
				        		if (active_tool > 0 && active_tool < 999990) {
				        			// drop current tool
				            		int old_tool = active_tool;
				            		// change to probe tool
				            		this->fill_drop_scripts(old_tool);
				        		}
			            		this->fill_pick_scripts(0, active_tool <= 0);
			            		this->fill_cali_scripts(true, false);
			            	}
			            	else	//Manual Tool Change
			            	{
			            		this->target_tool = 0;
			            		this->fill_change_scripts(0, true);
			            		this->fill_cali_scripts(true, true);
			            	}
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
			            	gcode->stream->printf("goto x and y clearance first\r\n");
			            	if(THEKERNEL->factory_set->FuncSetting & (1<<0) )
			            	{
			            		if (zprobe_abs) {
    								char buff[100];
    								// lift z to clearance position with fast speed
									snprintf(buff, sizeof(buff), "G53 G0 Z%.3f", THEROBOT->from_millimeters(this->clearance_z));
									this->script_queue.push(buff);

    								// Avoid the position of the chuck
									snprintf(buff, sizeof(buff), "G53 G90 G0 Y%.3f", THEROBOT->from_millimeters(this->clearance_y));
									this->script_queue.push(buff);
									
									// goto x and y clearance
									snprintf(buff, sizeof(buff), "G53 G90 G0 X%.3f", THEROBOT->from_millimeters(this->clearance_x));
									this->script_queue.push(buff);
			            		}
			            		else
			            		{
			            			this->fill_goto_origin_scripts(x_path_pos, y_path_pos);
			            		}
			            	}
			            	else
			            	{
			            		this->fill_goto_origin_scripts(x_path_pos, y_path_pos);
			            	}
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
			if (gcode->has_letter('A')) {
				position_a = gcode->get_value('A');
			}
			if (gcode->has_letter('B')) {
				position_b = gcode->get_value('B');
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
				for (int wcs_index = 0; wcs_index < 6; wcs_index++){
					THEKERNEL->streams->printf("EEPRROM Data: G5%d: %1.3f, %1.3f, %1.3f, %1.3f | R:%1.3f\n", wcs_index + 4, THEKERNEL->eeprom_data->WCScoord[wcs_index][0] , THEKERNEL->eeprom_data->WCScoord[wcs_index][1] , THEKERNEL->eeprom_data->WCScoord[wcs_index][2] , THEKERNEL->eeprom_data->WCScoord[wcs_index][3], THEKERNEL->eeprom_data->WCSrotation[wcs_index]);
				}
				THEKERNEL->streams->printf("EEPRROM Data: Tool Not Calibrated:%d\n", THEKERNEL->eeprom_data->tool_not_calibrated);
				THEKERNEL->streams->printf("EEPRROM Data: Last Active WCS:%d\n", THEKERNEL->eeprom_data->current_wcs);
			} else if (gcode->subcode == 2) {
				// Show EEPROM DATA
				THEKERNEL->erase_eeprom_data();
			}
		} else if ( gcode->m == 499 ) {
			if (gcode->subcode == 0 || gcode->subcode == 1) {
				THEKERNEL->streams->printf("tool:%d ref:%1.3f cur:%1.3f offset:%1.3f\n", active_tool, ref_tool_mz, cur_tool_mz, tool_offset);
			}
			else if (gcode->subcode == 2) 
			{
				THEKERNEL->streams->printf("probe (MCS) -- X:%1.1f Y:%1.1f Z:%1.1f\n", probe_mx_mm, probe_my_mm, probe_mz_mm);
				for (int i = 0; i <=  tool_number; i ++) {
					THEKERNEL->streams->printf("tool%d -- mx:%1.1f my:%1.1f mz:%1.1f\n", atc_tools[i].num, atc_tools[i].get_mx_mm(), atc_tools[i].get_my_mm(), atc_tools[i].get_mz_mm());
				}
			}
			else if (gcode->subcode == 3) 
			{
				this->beep_complete();
			} 
			else if (gcode->subcode == 4) 
			{
				this->beep_alarm();
			} 
			else if (gcode->subcode == 5) 
			{
				int tool_to_change = 1;
				if (gcode->has_letter('T')) {
					tool_to_change = gcode->get_value('T');
				}
				this->beep_tool_change(tool_to_change);

			}
		} else if ( gcode->m == 887 ) {
			THEROBOT->override_homed_check(false);
			THEKERNEL->streams->printf("Home Check Disabled\n");
		} else if ( gcode->m == 888 ) {
			THEROBOT->override_homed_check(true);
			THEKERNEL->streams->printf("Home Check Enabled\n");
		} else if ( gcode->m == 889 ) {
			// M889 - Display current tool configuration
			gcode->stream->printf("Tool Slots Configuration:\n");
			for (const auto& tool : this->atc_tools) {
				if (tool.is_valid()) {
					gcode->stream->printf("Tool %d: X=%.3f Y=%.3f Z=%.3f\n", 
						tool.num, tool.get_mx_mm(), tool.get_my_mm(), tool.get_mz_mm());
				}
			}
		} else if ( gcode->m == 890 ) {
			// M890 - Add or update custom tool slot and save to file
			// Usage: M890 T<tool_number> X<x_mm> Y<y_mm> [Z<z_mm>]
			// Z is optional - if not provided, uses the previous slot's Z position
			if (!gcode->has_letter('T') || !gcode->has_letter('X') || !gcode->has_letter('Y')) {
				gcode->stream->printf("ERROR: M890 requires T, X, and Y parameters\n");
				gcode->stream->printf("Usage: M890 T<tool_number> X<x_mm> Y<y_mm> [Z<z_mm>]\n");
				return;
			}
			
			int tool_num = (int)gcode->get_value('T');
			float x_mm = gcode->get_value('X');
			float y_mm = gcode->get_value('Y');
			float z_mm = gcode->has_letter('Z') ? gcode->get_value('Z') : NAN;
			
			this->add_custom_tool_slot(tool_num, x_mm, y_mm, z_mm);
			this->save_custom_tool_slots_to_file();
		} else if ( gcode->m == 891 ) {
			// M891 - Remove custom tool slot and save to file
			// Usage: M891 T<tool_number>
			if (!gcode->has_letter('T')) {
				gcode->stream->printf("ERROR: M891 requires T parameter\n");
				gcode->stream->printf("Usage: M891 T<tool_number>\n");
				return;
			}
			
			int tool_num = (int)gcode->get_value('T');
			this->remove_custom_tool_slot(tool_num);
			this->save_custom_tool_slots_to_file();
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
			
		if (THEKERNEL->is_suspending() || THEKERNEL->is_waiting() || THEKERNEL->is_tool_waiting()) {
			return;
		}

        void *return_value;
        bool ok = PublicData::get_value( player_checksum, is_playing_checksum, &return_value );
        if (ok) {
            bool playing = *static_cast<bool *>(return_value);
            if (this->playing_file && !playing) {
            	this->clear_script_queue();

				this->atc_status = NONE;
				// Clear TLO calibration flag to re-enable 3D probe crash detection
				bool tlo_calibrating = false;
				PublicData::set_value( zprobe_checksum, set_tlo_calibrating_checksum, &tlo_calibrating );
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
	        rapid_move(true, NAN, NAN, this->clearance_z, NAN, NAN);

	        // return to saved x and y position
	        rapid_move(true, last_pos[0], last_pos[1], NAN, NAN, NAN);
		}

        this->atc_status = NONE;
		// Clear TLO calibration flag to re-enable 3D probe crash detection
		bool tlo_calibrating = false;
		PublicData::set_value( zprobe_checksum, set_tlo_calibrating_checksum, &tlo_calibrating );

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
		rapid_move(true, NAN, NAN, this->clearance_z, NAN, NAN);
		// goto x and y clearance
		rapid_move(true, this->clearance_x, this->clearance_y, NAN, NAN, NAN);
		THECONVEYOR->wait_for_idle();
		THEROBOT->pop_state();
		g28_triggered = false;
    } else if (goto_position > -1) {
        rapid_move(true, NAN, NAN, this->clearance_z, NAN, NAN);
		if (goto_position == 0 || goto_position == 1) {
			// goto clearance
	        rapid_move(true, this->clearance_x, this->clearance_y, NAN, NAN, NAN);
		} else if (goto_position == 2) {
			// goto work origin
			// shrink A value first before move
    		float ma = THEROBOT->actuators[A_AXIS]->get_current_position();
    		if (fabs(ma) > 360) {
    			THEROBOT->reset_axis_position(fmodf(ma, 360.0), A_AXIS);
    		}    		
			// shrink B value first before move
//    		ma = THEROBOT->actuators[B_AXIS]->get_current_position();
//    		if (fabs(ma) > 360) {
//    			THEROBOT->reset_axis_position(fmodf(ma, 360.0), B_AXIS);
//    		}
			//rapid_move(false, 0, 0, NAN, 0, 0);
			rapid_move(false, 0, 0, NAN, 0, NAN);
		} else if (goto_position == 3) {
			// goto anchor 1
			rapid_move(true, this->anchor1_x, this->anchor1_y, NAN, NAN, NAN);
		} else if (goto_position == 4) {
			// goto anchor 2
			rapid_move(true, this->anchor1_x + this->anchor2_offset_x, this->anchor1_y + this->anchor2_offset_y, NAN, NAN, NAN);
		} else if (goto_position == 5) {
			// goto designative work position
			if (position_x < 8888 && position_y < 8888 && position_a < 88888888 && position_b < 88888888) {
				// shrink A value first before move
				float ma = THEROBOT->actuators[A_AXIS]->get_current_position();
				if (fabs(ma) > 360) {
					THEROBOT->reset_axis_position(fmodf(ma, 360.0), A_AXIS);
				}    		
				// shrink B value first before move
				/*ma = THEROBOT->actuators[B_AXIS]->get_current_position();
				if (fabs(ma) > 360) {
					THEROBOT->reset_axis_position(fmodf(ma, 360.0), B_AXIS);
				}*/
				//rapid_move(false, position_x, position_y, NAN, fmodf(position_a, 360.0), fmodf(position_b, 360.0));
				rapid_move(false, position_x, position_y, NAN, fmodf(position_a, 360.0), NAN);
				// reset the A_AXIS position as target value
				THEROBOT->reset_axis_position(position_a, A_AXIS);
				// reset the B_AXIS position as target value
				THEROBOT->reset_axis_position(position_b, B_AXIS);
						}
			else if (position_x < 8888 && position_y < 8888 && position_a < 88888888) {
				// shrink A value first before move
	    		float ma = THEROBOT->actuators[A_AXIS]->get_current_position();
	    		if (fabs(ma) > 360) {
	    			THEROBOT->reset_axis_position(fmodf(ma, 360.0), A_AXIS);
	    		}    		
				rapid_move(false, position_x, position_y, NAN, fmodf(position_a, 360.0), NAN);
				// reset the A_AXIS position as target value
				THEROBOT->reset_axis_position(position_a, A_AXIS);
			}
			else if (position_x < 8888 && position_y < 8888 && position_b < 88888888) {
				// shrink B value first before move
	    		/*float ma = THEROBOT->actuators[B_AXIS]->get_current_position();
	    		if (fabs(ma) > 360) {
	    			THEROBOT->reset_axis_position(fmodf(ma, 360.0), B_AXIS);
	    		}    		
				rapid_move(false, position_x, position_y, NAN, NAN, fmodf(position_b, 360.0));
				// reset the B_AXIS position as target value
				THEROBOT->reset_axis_position(position_b, B_AXIS);*/
			}
			else if(position_x < 8888 && position_y < 8888 )
			{
				rapid_move(false, position_x, position_y, NAN, NAN, NAN);
			}
			else if(position_a < 88888888 && position_b < 88888888 )
			{
				// shrink A value first before move
	    		float ma = THEROBOT->actuators[A_AXIS]->get_current_position();
	    		if (fabs(ma) > 360) {
	    			THEROBOT->reset_axis_position(fmodf(ma, 360.0), A_AXIS);
	    		}    		
				// shrink B value first before move
	    		/*ma = THEROBOT->actuators[B_AXIS]->get_current_position();
	    		if (fabs(ma) > 360) {
	    			THEROBOT->reset_axis_position(fmodf(ma, 360.0), B_AXIS);
	    		}    	*/	
				//rapid_move(false, NAN, NAN, NAN, fmodf(position_a, 360.0), fmodf(position_b, 360.0));
				rapid_move(false, NAN, NAN, NAN, fmodf(position_a, 360.0), NAN);
				// reset the A_AXIS position as target value
				THEROBOT->reset_axis_position(position_a, A_AXIS);
				// reset the B_AXIS position as target value
				THEROBOT->reset_axis_position(position_b, B_AXIS);
			}
			else if(position_a < 88888888)
			{
				// shrink A value first before move
	    		float ma = THEROBOT->actuators[A_AXIS]->get_current_position();
	    		if (fabs(ma) > 360) {
	    			THEROBOT->reset_axis_position(fmodf(ma, 360.0), A_AXIS);
	    		}    		
				rapid_move(false, NAN, NAN, NAN, fmodf(position_a, 360.0), NAN);
				// reset the A_AXIS position as target value
				THEROBOT->reset_axis_position(position_a, A_AXIS);
			}
			else if(position_b < 88888888)
			{
				// shrink B value first before move
	    		/*float ma = THEROBOT->actuators[B_AXIS]->get_current_position();
	    		if (fabs(ma) > 360) {
	    			THEROBOT->reset_axis_position(fmodf(ma, 360.0), B_AXIS);
	    		}    		
				rapid_move(false, NAN, NAN, NAN, NAN, fmodf(position_b, 360.0));
				// reset the B_AXIS position as target value
				THEROBOT->reset_axis_position(position_b, B_AXIS);*/
			}
		} else if (goto_position == 6) {
			// goto designative machine position
			if (position_x < 8888 && position_y < 8888 && position_a < 88888888 && position_b < 88888888) {
				// shrink A value first before move
	    		float ma = THEROBOT->actuators[A_AXIS]->get_current_position();
	    		if (fabs(ma) > 360) {
	    			THEROBOT->reset_axis_position(fmodf(ma, 360.0), A_AXIS);
	    		}    		
				// shrink B value first before move
	    		/*ma = THEROBOT->actuators[B_AXIS]->get_current_position();
	    		if (fabs(ma) > 360) {
	    			THEROBOT->reset_axis_position(fmodf(ma, 360.0), B_AXIS);
	    		}*/   		
				//rapid_move(true, position_x, position_y, NAN, fmodf(position_a, 360.0), fmodf(position_b, 360.0));
				rapid_move(true, position_x, position_y, NAN, fmodf(position_a, 360.0), NAN);
				// reset the A_AXIS position as target value
				THEROBOT->reset_axis_position(position_a, A_AXIS);
				// reset the B_AXIS position as target value
				THEROBOT->reset_axis_position(position_b, B_AXIS);
			}
			else if (position_x < 8888 && position_y < 8888 && position_a < 88888888) {
				// shrink A value first before move
	    		float ma = THEROBOT->actuators[A_AXIS]->get_current_position();
	    		if (fabs(ma) > 360) {
	    			THEROBOT->reset_axis_position(fmodf(ma, 360.0), A_AXIS);
	    		}    		
				rapid_move(true, position_x, position_y, NAN, fmodf(position_a, 360.0), NAN);
				// reset the A_AXIS position as target value
				THEROBOT->reset_axis_position(position_a, A_AXIS);
			}
			else if (position_x < 8888 && position_y < 8888 && position_b < 88888888) {
				// shrink B value first before move
	    		/*float ma = THEROBOT->actuators[B_AXIS]->get_current_position();
	    		if (fabs(ma) > 360) {
	    			THEROBOT->reset_axis_position(fmodf(ma, 360.0), B_AXIS);
	    		}*/
				//rapid_move(true, position_x, position_y, NAN, NAN, fmodf(position_b, 360.0));
				rapid_move(true, position_x, position_y, NAN, NAN, NAN);
				// reset the B_AXIS position as target value
				THEROBOT->reset_axis_position(position_b, B_AXIS);
			}
			else if(position_x < 8888 && position_y < 8888)
			{
				rapid_move(true, position_x, position_y, NAN, NAN, NAN);
			}
			else if(position_a < 88888888 && position_b < 88888888)
			{
				// shrink A value first before move
	    		float ma = THEROBOT->actuators[A_AXIS]->get_current_position();
	    		if (fabs(ma) > 360) {
	    			THEROBOT->reset_axis_position(fmodf(ma, 360.0), A_AXIS);
	    		}    		
				// shrink B value first before move
	    		/*ma = THEROBOT->actuators[B_AXIS]->get_current_position();
	    		if (fabs(ma) > 360) {
	    			THEROBOT->reset_axis_position(fmodf(ma, 360.0), B_AXIS);
	    		}*/
				//rapid_move(true, NAN, NAN, NAN, fmodf(position_a, 360.0), fmodf(position_b, 360.0));
				rapid_move(true, NAN, NAN, NAN, fmodf(position_a, 360.0), NAN);
				// reset the A_AXIS position as target value
				THEROBOT->reset_axis_position(position_a, A_AXIS);
				// reset the B_AXIS position as target value
				THEROBOT->reset_axis_position(position_b, B_AXIS);
			}
			else if(position_a < 88888888)
			{
				// shrink A value first before move
	    		float ma = THEROBOT->actuators[A_AXIS]->get_current_position();
	    		if (fabs(ma) > 360) {
	    			THEROBOT->reset_axis_position(fmodf(ma, 360.0), A_AXIS);
	    		}    		
				rapid_move(true, NAN, NAN, NAN, fmodf(position_a, 360.0), NAN);
				// reset the A_AXIS position as target value
				THEROBOT->reset_axis_position(position_a, A_AXIS);
			}
			else if(position_b < 88888888)
			{
				// shrink B value first before move
	    		/*float ma = THEROBOT->actuators[B_AXIS]->get_current_position();
	    		if (fabs(ma) > 360) {
	    			THEROBOT->reset_axis_position(fmodf(ma, 360.0), B_AXIS);
	    		}    		
				rapid_move(true, NAN, NAN, NAN, NAN, fmodf(position_b, 360.0));
				// reset the B_AXIS position as target value
				THEROBOT->reset_axis_position(position_b, B_AXIS);*/
			}
		}
		position_x = 8888;
		position_y = 8888;
		position_a = 88888888;
		position_b = 88888888;
		goto_position = -1;
    }
}

// issue a coordinated move directly to robot, and return when done
// Only move the coordinates that are passed in as not nan
// NOTE must use G53 to force move in machine coordinates and ignore any WCS offsets
void ATCHandler::rapid_move(bool mc, float x, float y, float z, float a, float b)
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
    
    if(!isnan(a)) {
        size_t n= strlen(cmd);
        //snprintf(&cmd[n], CMDLEN-n, " A%1.3f", THEROBOT->from_millimeters(a));
        snprintf(&cmd[n], CMDLEN-n, " A%1.3f", a);
    }
    if(!isnan(b)) {
        size_t n= strlen(cmd);
        //snprintf(&cmd[n], CMDLEN-n, " B%1.3f", THEROBOT->from_millimeters(b));
        snprintf(&cmd[n], CMDLEN-n, " B%1.3f", b);
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
		struct tool_status *t= static_cast<tool_status*>(pdr->get_data_ptr());
		t->active_tool = this->active_tool;
		t->target_tool = this->target_tool;
		t->ref_tool_mz = this->ref_tool_mz;
		t->cur_tool_mz = this->cur_tool_mz;
		if (this->active_tool >= 0) { 
            t->tool_offset = this->tool_offset;  
    	}else{
            t->tool_offset = 0.0; 
		}
		pdr->set_taken();
    } else if (pdr->second_element_is(get_atc_pin_status_checksum)) {
        char *data = static_cast<char *>(pdr->get_data_ptr());
        // cover endstop
        data[0] = (char)this->atc_home_info.pin.get();
        data[1] = (char)this->detector_info.detect_pin.get();
        pdr->set_taken();
    } else if (pdr->second_element_is(get_atc_clamped_status_checksum)) {
		uint8_t* data = static_cast<uint8_t*>(pdr->get_data_ptr());
		*data = static_cast<uint8_t>(this->atc_home_info.clamp_status); //0 unhomed, 1 clamped, 2 unclamped
	} else if (pdr->second_element_is(get_machine_offsets_checksum)) {
		struct machine_offsets *m = static_cast<machine_offsets*>(pdr->get_data_ptr());
		m->anchor1_x = this->anchor1_x;
		m->anchor1_y = this->anchor1_y;
		m->anchor2_offset_x = this->anchor2_offset_x;
		m->anchor2_offset_y = this->anchor2_offset_y;
		m->anchor_width = this->anchor_width;
		m->rotation_offset_x = this->rotation_offset_x;
		m->rotation_offset_y = this->rotation_offset_y;
		m->rotation_offset_z = this->rotation_offset_z;
		m->rotation_width = this->rotation_width;
		m->clearance_z = this->clearance_z;
		pdr->set_taken();
	}
}

void ATCHandler::on_set_public_data(void* argument)
{
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);

    if(!pdr->starts_with(atc_handler_checksum)) return;

    if(pdr->second_element_is(set_ref_tool_mz_checksum)) {
        // update eeprom data if needed
        if (this->ref_tool_mz != THEKERNEL->eeprom_data->REFMZ) {
        	THEKERNEL->eeprom_data->REFMZ = this->ref_tool_mz;
		    THEKERNEL->write_eeprom_data();
        }
        pdr->set_taken();
    } else if (pdr->second_element_is(abort_checksum)) {
		this->abort();
		pdr->set_taken();
	}
	
	if(CARVERA_AIR == THEKERNEL->factory_set->MachineModel)
	{
		if(pdr->second_element_is(set_job_complete_checksum)) 
		{
	        this->beep_complete();
	        pdr->set_taken();
	    }
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


// Called every 100ms in an ISR
uint32_t ATCHandler::beep_beep(uint32_t dummy)
{
	bool b_false = false, b_true = true;
	switch(beep_state)
	{		
		case BP_COMPLETE:	
			if(this->beep_count == 0)		// beep 1
			{
    			PublicData::set_value(switch_checksum, beep_checksum, state_checksum, &b_true);
			}
			else if(this->beep_count == 3)
			{
    			PublicData::set_value(switch_checksum, beep_checksum, state_checksum, &b_false);
			}
			else if(this->beep_count == 5)	// beep 2
			{
    			PublicData::set_value(switch_checksum, beep_checksum, state_checksum, &b_true);
			}
			else if(this->beep_count == 8)
			{
    			PublicData::set_value(switch_checksum, beep_checksum, state_checksum, &b_false);
			}
			else if(this->beep_count > 8)
			{
				this->beep_count = 8;
				this->beep_state = BP_SLEEP;
			}			
			break;
		case BP_ALARM:	
			if(this->beep_count == 0)		// beep 1
			{
    			PublicData::set_value(switch_checksum, beep_checksum, state_checksum, &b_true);
			}
			else if(this->beep_count == 5)
			{
    			PublicData::set_value(switch_checksum, beep_checksum, state_checksum, &b_false);
			}
			else if(this->beep_count == 15)	// beep 2
			{
    			PublicData::set_value(switch_checksum, beep_checksum, state_checksum, &b_true);
			}
			else if(this->beep_count == 20)
			{
    			PublicData::set_value(switch_checksum, beep_checksum, state_checksum, &b_false);
			}
			else if(this->beep_count > 20)
			{
				this->beep_count = 20;
				this->beep_state = BP_SLEEP;
			}			
			break;
		case BP_ERROR:	
			if(this->beep_count == 0)		// beep 1
			{
    			PublicData::set_value(switch_checksum, beep_checksum, state_checksum, &b_true);
			}
			else if(this->beep_count == 5)
			{
    			PublicData::set_value(switch_checksum, beep_checksum, state_checksum, &b_false);
			}
			else if(this->beep_count == 15)	// beep 2
			{
    			PublicData::set_value(switch_checksum, beep_checksum, state_checksum, &b_true);
			}
			else if(this->beep_count == 20)
			{
    			PublicData::set_value(switch_checksum, beep_checksum, state_checksum, &b_false);
			}
			else if(this->beep_count == 30)	// beep 3
			{
    			PublicData::set_value(switch_checksum, beep_checksum, state_checksum, &b_true);
			}
			else if(this->beep_count == 35)
			{
    			PublicData::set_value(switch_checksum, beep_checksum, state_checksum, &b_false);
			}
			else if(this->beep_count > 35)
			{
				this->beep_count = 35;
				this->beep_state = BP_SLEEP;
			}			
			break;
		case BP_TOOL:
			if(this->beep_count == 0)		// beep 1
			{
    			PublicData::set_value(switch_checksum, beep_checksum, state_checksum, &b_true);
			}
			else if(this->beep_count == 3)
			{
    			PublicData::set_value(switch_checksum, beep_checksum, state_checksum, &b_false);
			}
			else if(this->beep_count == 5)	// beep 2
			{
    			PublicData::set_value(switch_checksum, beep_checksum, state_checksum, &b_true);
			}
			else if(this->beep_count == 8)
			{
    			PublicData::set_value(switch_checksum, beep_checksum, state_checksum, &b_false);
			}
			else if(this->beep_count == 10)	// beep 3
			{
    			PublicData::set_value(switch_checksum, beep_checksum, state_checksum, &b_true);
			}
			else if(this->beep_count == 13)
			{
    			PublicData::set_value(switch_checksum, beep_checksum, state_checksum, &b_false);
			}
			else if(this->beep_count > 15)
			{
				this->beep_count = 15;
				this->beep_state = BP_SLEEP;
			}			
			break;
	}	
	
	this->beep_count ++;	
    return 0;
}
void ATCHandler::beep_complete() {
	this->beep_state = BP_COMPLETE;
	this->beep_count = 0;
}

void ATCHandler::beep_alarm() {
	this->beep_state = BP_ALARM;
	this->beep_count = 0;
}

void ATCHandler::beep_error() {
	this->beep_state = BP_ERROR;
	this->beep_count = 0;
}

void ATCHandler::beep_tool_change(int tool) {
	this->beep_state = BP_TOOL;
	this->beep_count = 0;
}


