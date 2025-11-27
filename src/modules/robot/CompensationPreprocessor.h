#pragma once

#include <array>
#include <vector>
#include <queue>
#include <cmath>
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "libs/StreamOutputPool.h"
#include "libs/StreamOutput.h"
#include "modules/communication/utils/Gcode.h"
#include "CompensationTypes.h"

class Robot; // Forward declaration instead of including Robot.h

class CompensationPreprocessor {
public:
    using CompSide = Compensation::Side;

    // Motion mode enum to avoid dependency on Robot's internal enum
    enum MotionMode {
        MOTION_NONE,
        MOTION_SEEK,     // G0
        MOTION_LINEAR,   // G1
        MOTION_CW_ARC,   // G2
        MOTION_CCW_ARC   // G3
    };

    // Lightweight structure to store parsed move data for buffering
    // Memory-efficient: ~80 bytes per move vs. ~300+ bytes for full Gcode object
    struct ParsedMove {
        // Coordinates in G-code space (mm, after unit conversion, before WCS transform)
        float xyz[3];           // X, Y, Z endpoint (use NAN for unspecified axes)
        float ijk[3];           // I, J, K offsets for arcs (NAN if not arc)
        
        // Move metadata
        MotionMode mode;        // G0, G1, G2, or G3
        float feed_rate;        // F value (mm/min), NAN if not specified
        unsigned int line_num;  // G-code line number for debugging
        
        // Computed values (cached for performance)
        float length_2d;        // XY distance (computed once)
        float direction[2];     // XY unit vector (computed once)
        
        // Flags
        bool valid;             // Is this move valid/initialized?
        
        // Constructor for easy initialization
        ParsedMove() : mode(MOTION_NONE), feed_rate(NAN), line_num(0), 
                       length_2d(0), valid(false) {
            xyz[0] = xyz[1] = xyz[2] = NAN;
            ijk[0] = ijk[1] = ijk[2] = NAN;
            direction[0] = direction[1] = 0;
        }
    };

    CompensationPreprocessor();
    ~CompensationPreprocessor();
    
    // ============================================================================
    // Main API - Called by Robot
    // ============================================================================
    
    // Buffer a move for preprocessing (called when G-code arrives)
    // Returns true if the move was buffered, false if it should be processed immediately
    bool buffer_move(const ParsedMove& move);
    
    // Get the next compensated move (returns false if buffer doesn't have enough moves)
    bool get_compensated_move(ParsedMove& output_move);
    
    // Check if there are buffered moves waiting
    bool has_buffered_moves() const { return buffer_count > 0; }
    int buffer_size() const { return buffer_count; }
    
    // Flush buffered moves one at a time (call on G40, M2, program end, etc.)
    // Returns true if a flushed move was retrieved, false if buffer is empty
    bool get_flushed_move(ParsedMove& output_move);
    
    // DEPRECATED: Old flush using std::queue (causes heap allocation)
    void flush_buffer(std::queue<ParsedMove>& flushed_moves);
    
    // Compensation control
    void enable_compensation(CompSide side, float diameter, const float current_position[3]);
    void disable_compensation();
    bool is_active() const { return comp_side != Compensation::NONE; }
    CompSide get_side() const { return comp_side; }
    float get_radius() const { return comp_radius; }

private:
    // ============================================================================
    // Circular Buffer Helpers (inline for performance)
    // ============================================================================
    inline bool buffer_is_full() const { return buffer_count >= MAX_BUFFER_SIZE; }
    inline bool buffer_is_empty() const { return buffer_count == 0; }
    
    inline void buffer_push(const ParsedMove& move) {
        move_buffer[buffer_head] = move;
        buffer_head = (buffer_head + 1) % MAX_BUFFER_SIZE;
        buffer_count++;
    }
    
    inline void buffer_pop() {
        buffer_tail = (buffer_tail + 1) % MAX_BUFFER_SIZE;
        buffer_count--;
    }
    
    inline const ParsedMove& buffer_at(int index) const {
        return move_buffer[(buffer_tail + index) % MAX_BUFFER_SIZE];
    }
    
    inline void buffer_clear() {
        buffer_head = buffer_tail = buffer_count = 0;
    }
    // ============================================================================
    // Lookahead Buffer Management
    // ============================================================================
    static const int LOOKAHEAD_DEPTH = 3;  // Need 3 moves for proper corner calculation
    static const int MAX_BUFFER_SIZE = 10; // Safety limit to prevent runaway memory usage
    
    // Fixed circular buffer (NO heap allocation - embedded system safety)
    ParsedMove move_buffer[MAX_BUFFER_SIZE];
    int buffer_head;  // Write index
    int buffer_tail;  // Read index
    int buffer_count; // Number of elements
    
    // Current compensation state
    CompSide comp_side;
    float comp_radius;
    
    // Track last processed position (compensated coordinates for next geometry calc)
    float last_position[3];
    // Track uncompensated position (G-code space endpoint of last move)
    float last_uncompensated_position[3];
    bool first_move_after_enable;
    
    // ============================================================================
    // Core Compensation Algorithms
    // ============================================================================
    
    // Calculate simple perpendicular offset for straight lines
    void calculate_perpendicular_offset(
        const ParsedMove& move,
        const float start_pos[2],
        float output_xy[2]
    ) const;
    
    // Calculate corner intersection between two moves
    bool calculate_corner_intersection(
        const ParsedMove& move1,
        const ParsedMove& move2,
        float intersection_xy[2]
    ) const;
    
    // Determine if corner is inside or outside
    bool is_inside_corner(
        const ParsedMove& move1,
        const ParsedMove& move2
    ) const;
    
    // Handle arc compensation (modify I,J,K offsets)
    void compensate_arc(ParsedMove& arc_move) const;
    
    // Check if move creates singularity (impossible geometry)
    bool is_singularity(
        const ParsedMove& move1,
        const ParsedMove& move2
    ) const;
    
    // ============================================================================
    // Geometry Utilities
    // ============================================================================
    
    // Calculate 2D distance and direction for a move
    void calculate_move_geometry(ParsedMove& move, const float start_pos[2]);
    
    // Calculate angle between two direction vectors
    float calculate_angle_between(
        const float dir1[2],
        const float dir2[2]
    ) const;
    
    // Cross product for determining turn direction
    float cross_product_2d(
        const float v1[2],
        const float v2[2]
    ) const;
    
    // ============================================================================
    // Debug Helpers
    // ============================================================================
    void print_move(const ParsedMove& move, const char* prefix) const;
    void print_buffer_state() const;
};
