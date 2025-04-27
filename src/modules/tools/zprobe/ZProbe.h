/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ZPROBE_H_
#define ZPROBE_H_

#include "Module.h"
#include "Pin.h"
#include <fastmath.h>

#include <vector>

// defined here as they are used in multiple files
#define zprobe_checksum            CHECKSUM("zprobe")
#define leveling_strategy_checksum CHECKSUM("leveling-strategy")

class StepperMotor;
class Gcode;
class StreamOutput;
class LevelingStrategy;

struct probe_parameters{
    float x_axis_distance;
    float y_axis_distance;
    float z_axis_distance;
    float x_rotated_x;
    float x_rotated_y;
    float y_rotated_x;
    float y_rotated_y;
    float rotation_angle;
    float rotation_angle_mcs;
    float tool_dia;
    float half_tool_dia_rotated_x_x;
    float half_tool_dia_rotated_x_y;
    float half_tool_dia_rotated_y_x;
    float half_tool_dia_rotated_y_y;
    float half_tool_dia_x;
    float half_tool_dia_y;
    float half_tool_dia_z;
    float probe_height;
    float side_depth;
    float feed_rate;
    float rapid_rate;
    float slowZprobeRate;
    float retract_distance;
    float clearance_height;
    float clearance_world_pos;
    float visualize_path_distance;
    float rotation_offset_per_probe;
    float extra_probe_distance;
    int repeat;
    int probe_g38_subcode;
    int save_position;
    bool invert_probe; 
};

struct xy_output_coordinates{
    float x_positive_x_out;
    float x_positive_y_out;
    float x_negative_x_out;
    float x_negative_y_out;
    float y_positive_x_out;
    float y_positive_y_out;
    float y_negative_x_out;
    float y_negative_y_out;
    float z_negative_z_out;
    float origin_x;
    float origin_y;
    float origin_z;
};

class ZProbe: public Module
{

public:
    ZProbe() : invert_override(false),invert_probe(false) {
        probe_calibration_safety_margin = 0.1F;
        reset_probe_tracking();  
    };

    virtual ~ZProbe() {};

    void on_module_loaded();
    void on_gcode_received(void *argument);

    bool check_last_probe_ok();
    bool run_probe(float& mm, float feedrate, float max_dist= -1, bool reverse= false);
    bool run_probe_return(float& mm, float feedrate, float max_dist= -1, bool reverse= false);
    bool doProbeAt(float &mm, float x, float y);

    void coordinated_move(float x, float y, float z, float feedrate, bool relative=false);
    void home();

    bool getProbeStatus() { return this->pin.get(); }
    float getSlowFeedrate() const { return slow_feedrate; }
    float getFastFeedrate() const { return fast_feedrate; }
    float getProbeHeight() const { return probe_height; }
    float getMaxZ() const { return max_z; }

private:
    void config_load();
    bool probe_XYZ(Gcode *gcode);
    void rotate(int axis, float axis_distance, float *y_x, float *y_y, float rotation_angle);
    void rotateXY(float x_in = NAN, float y_in = NAN, float *x_out = nullptr, float *y_out = nullptr, float rotation_angle = 0);
    float get_xyz_move_length(float x, float y, float z);
    void fast_slow_probe_sequence( int axis, int direction);
    int xy_probe_move_alarm_when_hit(int direction, int probe_g38_subcode, float x, float y, float feed_rate);
    void z_probe_move_with_retract(int probe_g38_subcode, float z, float clearance_height, float feed_rate);
    bool parse_parameters(Gcode *gcode, bool override_probe_check = false);
    void init_parameters_and_out_coords();
    void probe_bore(bool calibration = false);
    void probe_boss(bool calibration = false);
    void probe_insideCorner();
    void probe_outsideCorner();
    void probe_axisangle();
    void calibrate_probe_bore();
    void calibrate_probe_boss();
    void single_axis_probe_double_tap();
    void calibrate_Z(Gcode *gc);
    uint32_t read_probe(uint32_t dummy);
    uint32_t read_calibrate(uint32_t dummy);
    void on_get_public_data(void* argument);
    uint32_t probe_doubleHit(uint32_t dummy);
    void reset_probe_tracking();
    bool is_probe_tool();

    float slow_feedrate;
    float fast_feedrate;
    float return_feedrate;
    float probe_height;
    float max_z;
    bool tool_0_3axis;
    float dwell_before_probing;

    Gcode* gcodeBuffer;
    char buff[100];
    probe_parameters param;
    xy_output_coordinates out_coords;

    Pin pin;
    Pin calibrate_pin;
    std::vector<LevelingStrategy*> strategies;
    uint16_t debounce_ms;
	volatile uint16_t debounce, cali_debounce;

    uint32_t probe_trigger_time;

    volatile bool probing;
    volatile bool calibrating;
    volatile bool probe_detected;
    volatile bool calibrate_detected;


    bool bfirstHitDetected  = false;
    bool bNoHited  = false;
    bool bDoubleHited  = false;
    uint32_t probe_hit_time = 0;

    struct {
        bool is_delta:1;
        bool is_rdelta:1;
        bool reverse_z:1;
        bool invert_override:1;
        bool invert_probe:1;
    };
    
    // Tracking variables to prevent probe crashes
    // Track position when calibrate pin detected
    volatile float calibrate_pin_position;  
    // zprobe.calibrate_safety_margin
    float probe_calibration_safety_margin;
    // Z position when probe pin triggered        
    volatile float probe_pin_position;
    volatile float calibrate_current_z;
    volatile bool safety_margin_exceeded;
    volatile float distance_moved;
};

#endif /* ZPROBE_H_ */
