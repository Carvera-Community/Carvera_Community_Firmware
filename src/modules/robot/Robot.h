/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ROBOT_H
#define ROBOT_H

#include <string>
using std::string;
#include <string.h>
#include <functional>
#include <stack>
#include <vector>

#include "libs/Module.h"
#include "ActuatorCoordinates.h"
#include "nuts_bolts.h"
#include <fastmath.h>

class Gcode;
class BaseSolution;
class StepperMotor;

// 9 WCS offsets
#define MAX_WCS 9UL

class Robot : public Module {
    public:
        using wcs_t= std::tuple<float, float, float, float, float>;
        Robot();
        void on_module_loaded();
        void on_gcode_received(void* argument);

        void reset_axis_position(float position, int axis);
        void reset_axis_position(float x, float y, float z);
        void reset_actuator_position(const ActuatorCoordinates &ac);
        void reset_position_from_current_actuator_position();
        float get_seconds_per_minute() const { return seconds_per_minute; }
        float get_z_maxfeedrate() const { return this->max_speeds[Z_AXIS]; }
        float get_default_acceleration() const { return default_acceleration; }
        void loadToolOffset(const float offset[N_PRIMARY_AXIS]);
        void saveToolOffset(const float offset[N_PRIMARY_AXIS], const float cur_tool_mz);
        void set_probe_tool_not_calibrated(bool value);
        bool get_probe_tool_not_calibrated();
        float get_feed_rate() const;
        float get_s_value() const { return s_value; }
        void set_s_value(float s) { s_value= s; }
        float get_max_delta() const { return max_delta; }
        void set_max_delta(float delta) {max_delta = delta; }
        void  push_state();
        void  pop_state();
        void check_max_actuator_speeds();
        float to_millimeters( float value ) const { return this->inch_mode ? value * 25.4F : value; }
        float from_millimeters( float value) const { return this->inch_mode ? value/25.4F : value;  }
        float get_axis_position(int axis) const { return(this->machine_position[axis]); }
        void get_axis_position(float position[], size_t n= 3) const { memcpy(position, this->machine_position, n*sizeof(float)); }
        wcs_t get_axis_position() const { return wcs_t(machine_position[X_AXIS], machine_position[Y_AXIS], machine_position[Z_AXIS], machine_position[A_AXIS], machine_position[B_AXIS]); }
        void get_current_machine_position(float *pos) const;
        void print_position(uint8_t subcode, std::string& buf, bool ignore_extruders=false) const;
        uint8_t get_current_wcs() const { return current_wcs; }
        std::vector<wcs_t> get_wcs_state() const;
        std::tuple<float, float, float, uint8_t> get_last_probe_position() const { return last_probe_position; }
        void set_last_probe_position(std::tuple<float, float, float, uint8_t> p) { last_probe_position = p; }
        bool delta_move(const float delta[], float rate_mm_s, uint8_t naxis);
        uint8_t register_motor(StepperMotor*);
        uint8_t get_number_registered_motors() const {return n_motors; }
        uint8_t get_current_motion_mode() const {return current_motion_mode; }
        void clearLaserOffset();

        bool is_homed_all_axes();
        void override_homed_check(bool home_override_value);

        BaseSolution* arm_solution;                           // Selected Arm solution ( millimeters to step calculation )

        // gets accessed by Panel, Endstops, ZProbe
        std::vector<StepperMotor*> actuators;

        // set by a leveling strategy to transform the target of a move according to the current plan
        std::function<void(float*, bool, bool)> compensationTransform;
        // set by an active extruder, returns the amount to scale the E parameter by (to convert mm³ to mm)
        std::function<float(void)> get_e_scale_fnc;

        // Workspace coordinate systems
        wcs_t mcs2wcs(const wcs_t &pos) const { return mcs2selected_wcs(pos, current_wcs); }
        wcs_t mcs2selected_wcs(const wcs_t &pos, const size_t n) const;
        wcs_t mcs2wcs(const float *pos) const { return mcs2wcs(wcs_t(pos[X_AXIS], pos[Y_AXIS], pos[Z_AXIS], pos[A_AXIS], pos[B_AXIS])); }
        wcs_t mcs2selected_wcs(const float *pos, const size_t n) const { return mcs2selected_wcs(wcs_t(pos[X_AXIS], pos[Y_AXIS], pos[Z_AXIS], pos[A_AXIS], pos[B_AXIS]), n); }
        wcs_t wcs2mcs(const wcs_t &pos) const;
        wcs_t wcs2mcs(const float *pos) const { return wcs2mcs(wcs_t(pos[X_AXIS], pos[Y_AXIS], pos[Z_AXIS], pos[A_AXIS], pos[B_AXIS])); }

        void set_current_wcs_by_mpos(float x = NAN, float y = NAN, float z = NAN, float a = NAN, float b = NAN, float r = NAN);

        float r[MAX_WCS];

        struct {
            bool inch_mode:1;                                 // true for inch mode, false for millimeter mode ( default )
            bool absolute_mode:1;                             // true for absolute mode ( default ), false for relative mode
            bool e_absolute_mode:1;                           // true for absolute mode for E ( default ), false for relative mode
            bool next_command_is_MCS:1;                       // set by G53
            bool disable_segmentation:1;                      // set to disable segmentation
            bool disable_arm_solution:1;                      // set to disable the arm solution
            bool segment_z_moves:1;
            bool save_g92:1;                                  // save g92 on M500 if set
            bool save_g54:1;                                  // save WCS on M500 if set
            bool is_g123:1;
            bool soft_endstop_enabled:1;
            bool soft_endstop_halt:1;
            uint8_t plane_axis_0:2;                           // Current plane ( XY, XZ, YZ )
            uint8_t plane_axis_1:2;
            uint8_t plane_axis_2:2;
        };

    private:
        bool home_override = false;
        enum MOTION_MODE_T {
            NONE,
            SEEK, // G0
            LINEAR, // G1
            CW_ARC, // G2
            CCW_ARC // G3
        };

        void load_config();
        bool append_milestone(const float target[], float rate_mm_s, unsigned int line);
        bool append_line( Gcode* gcode, const float target[], float rate_mm_s, float delta_e);
        bool append_arc( Gcode* gcode, const float target[], const float offset[], float radius, bool is_clockwise );
        bool compute_arc(Gcode* gcode, const float offset[], const float target[], enum MOTION_MODE_T motion_mode);
        void process_move(Gcode *gcode, enum MOTION_MODE_T);
        bool is_homed(uint8_t i) const;

        float theta(float x, float y);
        void select_plane(uint8_t axis_0, uint8_t axis_1, uint8_t axis_2);
        void clearToolOffset();
        void setLaserOffset();
        int get_active_extruder() const;

        std::array<wcs_t, MAX_WCS> wcs_offsets; // these are persistent once saved with M500
        uint8_t current_wcs{0}; // 0 means G54 is enabled this is persistent once saved with M500
        wcs_t g92_offset;
        wcs_t tool_offset; // used for multiple extruders, sets the tool offset for the current extruder applied first
        float cos_r[MAX_WCS];
        float sin_r[MAX_WCS];
        std::tuple<float, float, float, uint8_t> last_probe_position{0,0,0,0};

        uint8_t current_motion_mode;
        using saved_state_t= std::tuple<float, float, bool, bool, bool, bool, uint8_t>; // save current feedrate and absolute mode, e absolute mode, inch mode, is_g123, current_wcs
        std::stack<saved_state_t> state_stack;               // saves state from M120

        float machine_position[k_max_actuators]; // Last requested position, in millimeters, which is what we were requested to move to in the gcode after offsets applied but before compensation transform
        float compensated_machine_position[k_max_actuators]; // Last machine position, which is the position before converting to actuator coordinates (includes compensation transform)

        float seek_rate;                                     // Current rate for seeking moves ( mm/min )
        float feed_rate;                                     // Current rate for feeding moves ( mm/min )
        float mm_per_line_segment;                           // Setting : Used to split lines into segments
        float mm_per_arc_segment;                            // Setting : Used to split arcs into segments
        float mm_max_arc_error;                              // Setting : Used to limit total arc segments to max error
        float delta_segments_per_second;                     // Setting : Used to split lines into segments for delta based on speed
        float seconds_per_minute;                            // for realtime speed change
        float default_acceleration;                          // the defualt accleration if not set for each axis
        float s_value;                                       // modal S value
        // 2024
        /*
        float s_values[8];                                   // block S values
		int   s_count;
		*/
        float arc_milestone[3];                              // used as start of an arc command
        float max_delta;

        float laser_module_offset_x;
		float laser_module_offset_y;
		float laser_module_offset_z;

		// Number of arc generation iterations by small angle approximation before exact arc trajectory
        // correction. This parameter may be decreased if there are issues with the accuracy of the arc
        // generations. In general, the default value is more than enough for the intended CNC applications
        // of grbl, and should be on the order or greater than the size of the buffer to help with the
        // computational efficiency of generating arcs.
        int arc_correction;                                  // Setting : how often to rectify arc computation
        float max_speeds[3];                                 // Setting : max allowable speed in mm/s for each axis
        float max_speed;                                     // Setting : maximum feedrate in mm/s as specified by F parameter
        bool probe_tool_not_calibrated;
        bool load_last_wcs;
        float soft_endstop_min[3], soft_endstop_max[3];

        uint8_t n_motors;                                    //count of the motors/axis registered

        // Used by Planner
        friend class Planner;
};


#endif
