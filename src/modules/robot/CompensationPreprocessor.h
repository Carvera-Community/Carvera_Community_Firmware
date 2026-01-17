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

class Gcode;

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
    bool is_active() const { return compensation_type != CompensationType::NONE; }
    
    /**
     * Buffer a G-code for processing
     * @param gcode - G-code to buffer (will be cloned)
     * @return true if buffered, false if buffer full
     */
    bool buffer_gcode(Gcode* gcode);
    
    /**
     * Get next compensated G-code
     * @return Pointer to compensated Gcode, or nullptr if buffer empty
     */
    Gcode* get_compensated_gcode();
    
    /**
     * Flush remaining buffered moves
     * Called when compensation is turned off (G40)
     */
    void flush();
    
    /**
     * Clear all buffered moves
     */
    void clear();
    
private:
    // Buffered G-code structure
    struct BufferedGcode {
        Gcode* gcode;          // Cloned G-code object
        float endpoint[3];     // Endpoint in XYZ
        float ijk[3];          // I/J/K for arcs
        bool has_ijk;          // True if arc move
        bool is_cw;            // True for G2, false for G3
        bool is_move;          // True if G0/G1/G2/G3
        float direction[3];    // Unit direction vector (for lines)
    };
    
    // Circular buffer
    static const int BUFFER_SIZE = 10;
    BufferedGcode buffer[BUFFER_SIZE];
    int buffer_head;
    int buffer_tail;
    int buffer_count;
    
    // Compensation state
    CompensationType compensation_type;
    float compensation_radius;
    float uncompensated_position[3];  // Track uncompensated position for I/J calculation
    
    // Helper functions
    bool buffer_has_space() const { return buffer_count < BUFFER_SIZE; }
    int buffer_next_index(int index) const { return (index + 1) % BUFFER_SIZE; }
    
    /**
     * Clone and extract G-code data
     * @param gcode - Original G-code
     * @param slot - Buffer slot to fill
     */
    void clone_and_extract(Gcode* gcode, BufferedGcode& slot);
    
    /**
     * Apply compensation to move coordinates
     * Looks ahead at next 2 moves for corner detection
     * @param index - Buffer index of move to compensate
     */
    void apply_compensation(int index);
    
    /**
     * Calculate perpendicular offset for straight line
     */
    void calculate_perpendicular_offset(
        const float endpoint[2],
        const float direction[2],
        float radius,
        bool is_left,
        float output[2]
    );
    
    /**
     * Calculate corner intersection
     */
    bool calculate_corner_intersection(
        const float corner_point[2],
        const float dir1[2],
        const float dir2[2],
        float radius,
        bool is_left,
        float output[2]
    );
    
    /**
     * Determine if corner is inside or outside
     */
    bool is_inside_corner(
        const float dir1[2],
        const float dir2[2],
        bool is_left
    );
    
    /**
     * Compensate arc endpoint
     */
    bool compensate_arc_endpoint(
        const float uncomp_start[2],
        float arc_endpoint[2],
        float arc_ij[2],
        float comp_radius,
        bool is_left,
        bool is_cw
    );
    
    /**
     * Modify G-code coordinates
     * Rebuilds G-code string with new X/Y/I/J values
     * @param gcode - G-code to modify
     * @param new_endpoint - New XYZ endpoint
     * @param new_ijk - New IJK values (for arcs)
     * @param has_ijk - True if arc move
     */
    void modify_gcode_coordinates(
        Gcode* gcode,
        const float new_endpoint[3],
        const float new_ijk[3],
        bool has_ijk
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
};

#endif // COMPENSATION_PREPROCESSOR_H
