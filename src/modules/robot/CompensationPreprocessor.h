/*
      This file is part of Smoothie (http://smoothieware.org/).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef COMPENSATION_PREPROCESSOR_H
#define COMPENSATION_PREPROCESSOR_H

#include "CompensationTypes.h"

#include <cmath>
#include <cstdint>

#ifndef CUTTER_COMPENSATION_TRACE_ENABLED
#define CUTTER_COMPENSATION_TRACE_ENABLED 0
#endif

#if CUTTER_COMPENSATION_TRACE_ENABLED
#define COMPENSATION_TRACE_PRINTF(stream, ...) do { (stream)->printf(__VA_ARGS__); } while (0)
#else
#define COMPENSATION_TRACE_PRINTF(stream, ...) do { } while (0)
#endif

class Gcode;
class StreamOutput;

/**
 * Cutter Compensation Preprocessor v2.0 - Bolt-On Architecture
 * 
 * Design Philosophy:
 * - Gcode-in, Gcode-out: Modifies G-code coordinates, not internal structures
 * - Single execution path: ALL moves go through Robot::process_move()
 * - Lookahead buffer: 3-move window for corner detection
 * - Circular buffer: No heap allocation (10 slots fixed)
 * - String reconstruction: Rebuild G-code from modified coordinates
 * 
 * Memory cost: ~2.2KB (10 slots × ~220 bytes/Gcode)
 * Code savings: -150 lines (removes duplicate transform logic)
 */
class CompensationPreprocessor {
public:
    struct LoadBalanceMetrics {
        uint32_t input_gcode_count;
        uint32_t generated_gcode_count;
        uint32_t served_gcode_count;
        uint32_t compute_count;
        uint32_t priming_wait_count;
        uint32_t load_balance_wait_count;
        uint32_t empty_serve_count;
        uint32_t sample_count;
        int32_t cumulative_ready_margin;
        int32_t min_ready_margin;
        int32_t max_ready_margin;
        uint8_t max_uncomp_depth;
        uint8_t max_comp_ready_depth;
    };

    CompensationPreprocessor();
    ~CompensationPreprocessor();
    
    /**
     * Enable/disable compensation
     * @param type - NONE (G40), LEFT (G41), or RIGHT (G42)
     * @param radius - Tool radius (D word value)
     */
    void set_compensation(CompensationType type, float radius);
    
    /**
     * Check if compensation is active
     */
    bool is_active() const { return comp_active; }

    /**
     * Get cumulative load-balance metrics for the dual-buffer pipeline.
     */
    const LoadBalanceMetrics& get_load_balance_metrics() const { return load_balance_metrics; }

    /**
     * Reset cumulative load-balance metrics without disturbing buffer state.
     */
    void reset_load_balance_metrics();

    /**
     * Print a human-readable load-balance report to the given stream.
     * Called once at the end of a G40 flush so the report appears in the
     * operator console without adding any per-move I/O overhead.
     */
    void print_load_balance_report(StreamOutput* stream) const;
    
    /**
     * Buffer a G-code for processing
        * @param gcode - G-code to buffer
        * @return Next compensated Gcode for this clock tick, or nullptr while priming
     */
        Gcode* buffer_gcode(Gcode* gcode);
    
    /**
     * Get next compensated G-code
     * @return Pointer to compensated Gcode, or nullptr if buffer empty
     */
    Gcode* get_compensated_gcode();
    
    /**
     * Get current buffer count
     * @return Number of moves currently buffered
     */
    int get_buffer_count() const { return uncomp_count; }
    
    /**
     * Flush remaining buffered moves
     * Called when compensation is turned off (G40)
     */
    void flush();
    
    /**
     * Clear all buffered moves
     */
    void clear();
    
    /**
     * Set initial uncompensated position
     * Called when compensation is activated (G41/G42) to initialize position tracking
     * @param position - Current machine position [X, Y, Z]
     */
    void set_initial_position(const float position[3]);
    
private:
    // ========================================================================
    // PHASE-LOCKED DUAL RING BUFFER ARCHITECTURE
    // ========================================================================
    
    // Uncompensated buffer: Stores programmed path
    struct UncompPoint {
        float x, y, z;
    };
    
    // Compensated buffer: Stores offset path coordinates and Gcode pointer
    struct CompPoint {
        float x, y, z;
        Gcode* gcode;  // Compensated Gcode to return to Robot.cpp
    };
    
    // Ring buffer size (3 slots for lookahead)
    static const int BUFFER_SIZE = 3;
    
    // Uncompensated buffer (source of truth for programmed path)
    UncompPoint uncomp_ring[BUFFER_SIZE];
    int uncomp_head;   // Next slot to write
    int uncomp_tail;   // Next slot to compute
    int uncomp_count;  // Occupied slots (0..3), stays 3 when full
    
    // Compensated buffer (computed offset path)
    CompPoint comp_ring[BUFFER_SIZE];
    int comp_head;     // Next slot to write compensated output
    int comp_tail;     // Next slot to serve/drain
    int comp_count;    // Logical fullness of comp ring
    int comp_ready_count; // Actual queued Gcode pointers waiting to be served
    
    // Compensation state
    CompensationType comp_type;    // LEFT (G41), RIGHT (G42), or NONE (G40)
    float comp_radius;              // Tool radius (D word value)
    bool comp_active;               // Is compensation active?
    bool is_flushing;               // True when flushing remaining moves
    bool pipeline_primed;           // True after initial double-compute priming
    bool first_output_pending;      // True until the first compensated point is computed
    bool has_initial_position;      // True once G41/G42 starting position is captured
    uint8_t last_g;                 // Last G-code number seen (for modal G-codes)
    float initial_position[3];      // WCS position captured when compensation activates
    LoadBalanceMetrics load_balance_metrics;
    
    // Helper functions
    int get_comp_head() const { return comp_head; }
    int get_comp_tail() const { return comp_tail; }
    void record_load_balance_sample();
    
    /**
     * Compute compensated coordinates and output
     * Phase-locked: Computes comp_ring[comp_head] using uncomp_ring data
     */
    void compute_and_output();

    /**
     * Compute the terminal compensated point during G40 flush.
     * Uses the last segment direction and offsets the final point.
     */
    bool compute_terminal_output();

    /**
     * Serve one compensated Gcode from the tail of the comp ring.
     * @param draining - When true, decrement ring counts for explicit flush draining.
     */
    Gcode* serve_compensated_gcode(bool draining);
    
    /**
     * Calculate perpendicular offset for Phase 2
     * @param prev - Previous point (for direction)
     * @param curr - Current point (to be offset)
     * @param output - Output array [x, y] for offset point
     * @return true if offset calculated, false if zero-length move
     */
    bool calculate_perpendicular_offset(
        const UncompPoint& prev,
        const UncompPoint& curr,
        float output[2]
    );

    /**
     * Calculate compensated corner point for B using A-B-C triplet.
     * Degenerate cases (A==B or B==C) reduce to a perpendicular offset.
     */
    bool calculate_corner_intersection(
        const UncompPoint& a,
        const UncompPoint& b,
        const UncompPoint& c,
        float output[2]
    );
    
    // Geometry utilities
    float cross_product_2d(const float v1[2], const float v2[2]) {
        return v1[0] * v2[1] - v1[1] * v2[0];
    }
    
    void normalize_vector(float v[3]) {
        float mag = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
        if (mag > 0.00001f) {
            v[0] /= mag;
            v[1] /= mag;
            v[2] /= mag;
        }
    }
    
    // ========================================================================
    // PHASE 2: TRACE OUTPUT FUNCTIONS FOR DUAL BUFFER VISIBILITY
    // ========================================================================
    void print_uncomp_buffered();
    void print_offset_calc(const UncompPoint& prev, const UncompPoint& curr, 
                          const float dir[2], const float normal[2], 
                          const float offset[2]);
    void print_output(const char* gcode_str);
    void print_buffer_state();
};

#endif // COMPENSATION_PREPROCESSOR_H
