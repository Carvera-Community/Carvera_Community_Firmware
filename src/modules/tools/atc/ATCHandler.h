#ifndef _ATCHANDLER_H
#define _ATCHANDLER_H

using namespace std;
#include "Module.h"
#include <vector>
#include <queue>
#include <cstdint>
#include <cmath>
#include "Pin.h"
#include "Gcode.h"

class ATCHandler : public Module
{
public:
    ATCHandler();

    void on_module_loaded();
    void on_gcode_received(void *argument);
    void on_get_public_data(void *argument);
    void on_set_public_data(void *argument);
    void on_main_loop( void* argument );
    void on_halt(void *argument);
    int get_active_tool() const { return active_tool; }
    void on_config_reload(void *argument);


private:
    typedef enum {
        NONE,
        CHANGE,
        FULL, 				// M6T?
        DROP, 				// M6T-1
        PICK, 				// M6T?
		CALI, 				// M491
		AUTOMATION			// M495
//		PROBE,				// M494
//		PROBE_PICK,			// M494
//		PROBE_FULL,			// M494
//		AUTOLEVEL,			// M495
//		AUTOLEVEL_PICK, 	// M495
//		AUTOLEVEL_FULL,		// M495
//		PROBELEVEL,			// M496
//		PROBELEVEL_PICK,	// M496
//		PROBELEVEL_FULL		// M496
    } ATC_STATUS;

    typedef enum {
    	UNHOMED,	// need to home first
		CLAMPED,	// status after home or clamp
		LOOSED,		// status after loose
    } CLAMP_STATUS;

	typedef enum {
    	BP_SLEEP,	// do nothing
		BP_ALARM,	
		BP_ERROR,	
		BP_COMPLETE,//job complete
		BP_TOOL, 	//change tools
    } BEEP_STATUS;
    
    ATC_STATUS atc_status;
    

    uint32_t read_endstop(uint32_t dummy);
    uint32_t read_detector(uint32_t dummy);
    uint32_t countdown_probe_laser(uint32_t dummy);
    uint32_t beep_beep(uint32_t dummy);

    void switch_probe_laser(bool state);

    // clamp actions
    void clamp_tool();
    void loose_tool();
    void home_clamp();

    // laser detect
    bool laser_detect();

    // probe check
    bool probe_detect();

    void set_inner_playing(bool inner_playing);
    bool get_inner_playing() const;
    void abort();

    // set tool offset afteer calibrating
    void set_tool_offset();

    //
    void fill_change_scripts(int new_tool, bool clear_z, int old_tool, float custom_TLO);
    void fill_drop_scripts(int old_tool);
    void fill_pick_scripts(int new_tool, bool clear_z);
    void fill_cali_scripts(bool is_probe, bool clear_z);

    //
    void fill_manual_drop_scripts(int old_tool);
    void fill_manual_pickup_scripts(int new_tool, bool clear_z, bool auto_calibrate, float custom_TLO);

    //
    void fill_margin_scripts(float x_pos, float y_pos, float x_pos_max, float y_pos_max);
    void fill_zprobe_scripts(float x_pos, float y_pos, float x_offset, float y_offset);
    void fill_zprobe_abs_scripts();
    void fill_xyzprobe_scripts(float tool_dia, float probe_height);

    //
    void set_tlo_by_offset(float z_axis_offset);

    void fill_autolevel_scripts(float x_pos, float y_pos, float x_size, float y_size, int x_grids, int y_grids, float height);
    void fill_goto_origin_scripts(float x_pos, float y_pos);

    void fill_calibrate_probe_anchor_scripts(bool invert_probe);
    void calibrate_anchor1(Gcode *gcode);
    void calibrate_anchor2(Gcode *gcode);
    void calibrate_a_axis_headstock(Gcode *gcode);
    void calibrate_a_axis_height(Gcode *gcode);
    void home_machine_with_pin(Gcode *gcode);
    void calibrate_set_value(Gcode *gcode);

    void clear_script_queue();

    void rapid_move(bool mc, float x, float y, float z, float a, float b);
    void beep_complete();
    void beep_alarm();
    void beep_tool_change(int tool);
    void beep_error();

    std::queue<string> script_queue;

    uint16_t debounce;
    bool atc_homing;
    bool detecting;

    bool playing_file;
    bool g28_triggered;
    
    uint16_t probe_laser_last;

    using atc_homing_info_t = struct {
        Pin pin;
        uint16_t debounce_ms;
        float max_travel;
        float retract;
        float homing_rate;
        float action_rate;
        float action_dist;

        struct {
            bool triggered:1;
            CLAMP_STATUS clamp_status;
        };
    };
    atc_homing_info_t atc_home_info;

    using detector_info_t = struct {
        Pin detect_pin;
        float detect_rate;
        float detect_travel;
        bool triggered;
    };
    detector_info_t detector_info;

    float safe_z_mm;
    float safe_z_empty_mm;
    float safe_z_offset_mm;
    float fast_z_rate;
    float slow_z_rate;
    float margin_rate;
    float probe_mx_mm;
    float probe_my_mm;
    float probe_mz_mm;
    float probe_fast_rate;
    float probe_slow_rate;
    float probe_retract_mm;
    float probe_height_mm;

    // Configurable probe position (absolute Machine Coordinate System)
    float probe_mcs_x;
    float probe_mcs_y;
    float probe_mcs_z;
    bool probe_position_configured;

    // One-off probe position offsets for M493.1 (temporary offsets for specific tools)
    float probe_oneoff_x;
    float probe_oneoff_y;
    float probe_oneoff_z;
    bool probe_oneoff_configured;

    float last_pos[3];

    float anchor1_x;
    float anchor1_y;
    float anchor2_offset_x;
    float anchor2_offset_y;
    float anchor_width;

    float rotation_offset_x;
    float rotation_offset_y;
    float rotation_offset_z;
    float rotation_width = 100;

    float toolrack_offset_x;
    float toolrack_offset_y;
    float toolrack_z;

    float clearance_x;
    float clearance_y;
    float clearance_z;

    bool skip_path_origin;

    struct atc_tool {
    	uint8_t num;    // Tool number (0-255)
    	int16_t mx_mm;  // Stored as 0.01mm units (e.g., -25000 = -250.00mm), -1 = invalid
    	int16_t my_mm;  // Stored as 0.01mm units, -1 = invalid
    	int16_t mz_mm;  // Stored as 0.01mm units, -1 = invalid
    	
    	// Default constructor to initialize to invalid state
    	atc_tool() : num(0), mx_mm(-1), my_mm(-1), mz_mm(-1) {}
    	
    	// Check if tool is valid (all coordinates are set, not -1)
    	bool is_valid() const {
    		return (mx_mm != -1 && my_mm != -1 && mz_mm != -1);
    	}
    	
    	// Helper functions to convert to/from float (in millimeters)
    	float get_mx_mm() const { return mx_mm / 100.0f; }
    	float get_my_mm() const { return my_mm / 100.0f; }
    	float get_mz_mm() const { return mz_mm / 100.0f; }
    	void set_mx_mm(float val) {
    		// Handle NaN and invalid values
    		if (std::isnan(val) || val > 1000.0f || val < -1000.0f) {
    			mx_mm = -1;
    			return;
    		}
    		float fixed = val * 100.0f;
    		// Clamp to int16_t range to prevent overflow (-327.68mm to 327.67mm)
    		if (fixed > 32767.0f) fixed = 32767.0f;
    		else if (fixed < -32768.0f) fixed = -32768.0f;
    		mx_mm = (int16_t)(fixed + (fixed >= 0 ? 0.5f : -0.5f));
    	}
    	void set_my_mm(float val) {
    		// Handle NaN and invalid values
    		if (std::isnan(val) || val > 1000.0f || val < -1000.0f) {
    			my_mm = -1;
    			return;
    		}
    		float fixed = val * 100.0f;
    		// Clamp to int16_t range to prevent overflow (-327.68mm to 327.67mm)
    		if (fixed > 32767.0f) fixed = 32767.0f;
    		else if (fixed < -32768.0f) fixed = -32768.0f;
    		my_mm = (int16_t)(fixed + (fixed >= 0 ? 0.5f : -0.5f));
    	}
    	void set_mz_mm(float val) {
    		// Handle NaN and invalid values
    		if (std::isnan(val) || val > 1000.0f || val < -1000.0f) {
    			mz_mm = -1;
    			return;
    		}
    		float fixed = val * 100.0f;
    		// Clamp to int16_t range to prevent overflow (-327.68mm to 327.67mm)
    		if (fixed > 32767.0f) fixed = 32767.0f;
    		else if (fixed < -32768.0f) fixed = -32768.0f;
    		mz_mm = (int16_t)(fixed + (fixed >= 0 ? 0.5f : -0.5f));
    	}
    };

    vector<struct atc_tool> atc_tools;

    int active_tool;
    int target_tool;
    int tool_number;
    int max_manual_tool_number;
    int lowest_tool_number;
    int goto_position;
    float position_x;
    float position_y;
    float position_a;
    float position_b;

    float ref_tool_mz;
    float cur_tool_mz;
    float tool_offset;
    int beep_state;
    int beep_count;

    // Tool slots functions
    void load_custom_tool_slots();
    bool is_custom_tool_defined(int tool_num);
    void add_custom_tool_slot(int tool_num, float x_mm, float y_mm, float z_mm);
    void remove_custom_tool_slot(int tool_num);
    void save_custom_tool_slots_to_file();

};

#endif /* _ATCHANDLER_H */
