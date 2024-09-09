#ifndef _ATCHANDLER_H
#define _ATCHANDLER_H

using namespace std;
#include "Module.h"
#include <vector>
#include <queue>
#include "Pin.h"

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

    ATC_STATUS atc_status;

    uint32_t read_endstop(uint32_t dummy);
    uint32_t read_detector(uint32_t dummy);
    uint32_t countdown_probe_laser(uint32_t dummy);

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

    // set tool offset afteer calibrating
    void set_tool_offset();

    //
    void fill_drop_scripts(int old_tool);
    void fill_pick_scripts(int new_tool, bool clear_z);
    void fill_cali_scripts(bool is_probe, bool clear_z);

    //
    void fill_margin_scripts(float x_pos, float y_pos, float x_pos_max, float y_pos_max);
    void fill_zprobe_scripts(float x_pos, float y_pos, float x_offset, float y_offset);
    void fill_zprobe_abs_scripts();
    void fill_xyzprobe_scripts(float tool_dia, float probe_height);
    void fill_autolevel_scripts(float x_pos, float y_pos, float x_size, float y_size, int x_grids, int y_grids, float height);
    void fill_goto_origin_scripts(float x_pos, float y_pos);

    void clear_script_queue();

    void rapid_move(bool mc, float x, float y, float z);

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

    float last_pos[3];

    float anchor1_x;
    float anchor1_y;
    float anchor2_offset_x;
    float anchor2_offset_y;

    float rotation_offset_x;
    float rotation_offset_y;
    float rotation_offset_z;

    float toolrack_offset_x;
    float toolrack_offset_y;
    float toolrack_z;

    float clearance_x;
    float clearance_y;
    float clearance_z;

    struct atc_tool {
    	int num;
    	float mx_mm;
    	float my_mm;
    	float mz_mm;
    };

    vector<struct atc_tool> atc_tools;

    int active_tool;
    int tool_number;
    int goto_position;
    float position_x;
    float position_y;

    float ref_tool_mz;
    float cur_tool_mz;
    float tool_offset;

};

#endif /* _ATCHANDLER_H */
