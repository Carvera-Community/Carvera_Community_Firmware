#pragma once

#include <array>
#include <vector>
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "Robot.h"
#include "Gcode.h"

class CompensationPreprocessor {
public:
    enum CompSide {
        NONE,
        LEFT,   // G41
        RIGHT   // G42
    };

    struct Move {
        float start[2];  // XY start position
        float end[2];    // XY end position
        bool is_arc;
        bool clockwise;  // For arcs
        float center[2]; // For arcs (I,J)
        float radius;    // For arcs
        int line_number;
    };

    CompensationPreprocessor();
    
    // Main entry point for preprocessing moves
    bool preprocess_move(Gcode* gcode, float* target, float* position);
    
    // Compensation control
    void enable_compensation(CompSide side, float diameter);
    void disable_compensation();
    bool is_active() const { return comp_side != NONE; }

private:
    // Move buffer for lookahead
    static const int LOOKAHEAD_SIZE = 3;
    std::vector<Move> move_buffer;
    
    // Current state
    CompSide comp_side;
    float comp_radius;
    
    // Geometric calculations
    void calculate_line_offset(const Move& move, float* output);
    void calculate_arc_offset(const Move& move, float* output);
    bool check_arc_validity(const Move& move);
    void calculate_corner_intersection(const Move& prev, const Move& next, float* output);
    
    // Buffer management
    void buffer_move(const Move& move);
    void flush_moves();
    
    // Geometric calculations
    void calculate_corner(const Move& prev, const Move& current, const Move& next, float* output);
    bool is_inside_corner(const Move& prev, const Move& next) const;
    void calculate_intersection(const Move& line1, const Move& line2, float* output);
    float calculate_corner_offset(const Move& prev, const Move& next) const;
    
    // Debug helpers
    void print_move(const Move& move, const char* prefix) const;
    
    // Handle arc move I,J offset adjustments
    void preprocess_arc_offsets(float offset[2], bool clockwise);
};