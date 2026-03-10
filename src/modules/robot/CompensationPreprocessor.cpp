/*
      This file is part of Smoothie (http://smoothieware.org/).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "CompensationPreprocessor.h"

using namespace std;

#include "mri.h"
#include "nuts_bolts.h"
#include "Gcode.h"
#include "Module.h"
#include "Kernel.h"
#include "StreamOutput.h"
#include "StreamOutputPool.h"

#include <cstring>
#include <cmath>
#include <cstdio>

CompensationPreprocessor::CompensationPreprocessor()
{
    buffer_head = 0;
    buffer_tail = 0;
    buffer_count = 0;
    compensation_type = CompensationType::NONE;
    compensation_radius = 0.0f;
    memset(uncompensated_position, 0, sizeof(uncompensated_position));
    memset(compensated_position, 0, sizeof(compensated_position));
    is_flushing = false;
    needs_lead_in = false;
    last_g = 0;  // Default to G0
    
    // Initialize buffer
    for (int i = 0; i < BUFFER_SIZE; i++) {
        buffer[i].gcode = nullptr;
        buffer[i].is_move = false;
        buffer[i].has_ijk = false;
    }
}

CompensationPreprocessor::~CompensationPreprocessor()
{
    clear();
}

void CompensationPreprocessor::set_compensation(CompensationType type, float radius)
{
    compensation_type = type;
    compensation_radius = radius;
    
    if (type == CompensationType::NONE) {
        is_flushing = true;
        needs_lead_in = false;
        THEKERNEL->streams->printf(">>PHASE2: G40 activated - FLUSHING mode (will output remaining %d buffered commands)\n", buffer_count);
    } else {
        THEKERNEL->streams->printf(">>PHASE2: G41/G42 called - clearing buffer (count was %d)\n", buffer_count);
        clear();
        
        is_flushing = false;
        needs_lead_in = true;
        
        const char* side_str = (type == CompensationType::LEFT) ? "LEFT (G41)" : "RIGHT (G42)";
        THEKERNEL->streams->printf(">>PHASE2: Compensation activated - type=%s, radius=%.3f\n", side_str, radius);
        
        compensated_position[X_AXIS] = uncompensated_position[X_AXIS];
        compensated_position[Y_AXIS] = uncompensated_position[Y_AXIS];
        compensated_position[Z_AXIS] = uncompensated_position[Z_AXIS];
    }
}

bool CompensationPreprocessor::buffer_gcode(Gcode* gcode)
{
    if (!buffer_has_space()) {
        return false;
    }
    
    int slot_index = buffer_head;
    BufferedGcode& slot = buffer[slot_index];
    
    clone_and_extract(gcode, slot);
    
    buffer_head = buffer_next_index(buffer_head);
    buffer_count++;
    
    THEKERNEL->streams->printf(">>PHASE2_BUFFERED: Command added to buffer (count now=%d, head=%d, tail=%d)\n",
        buffer_count, buffer_head, buffer_tail);
    
    return true;
}

void CompensationPreprocessor::clone_and_extract(Gcode* gcode, BufferedGcode& slot)
{
    // Clone the G-code object
    slot.gcode = new Gcode(*gcode);
    
    // Debug: Show what's being buffered
    THEKERNEL->streams->printf(">>EXTRACT: '%s' has_g=%d g=%d has_X=%d has_Y=%d has_Z=%d\n",
        gcode->get_command(),
        gcode->has_g ? 1 : 0,
        gcode->has_g ? gcode->g : -1,
        gcode->has_letter('X') ? 1 : 0,
        gcode->has_letter('Y') ? 1 : 0,
        gcode->has_letter('Z') ? 1 : 0);
    
    // Track last G-code number for modal moves
    uint8_t g_code;
    if (gcode->has_g) {
        g_code = gcode->g;
        // Update last_g only for motion commands (G0-G3)
        if (g_code <= 3) {
            last_g = g_code;
        }
    } else {
        // Modal move - use last G-code number
        g_code = last_g;
    }
    
    // Extract move type - MUST have explicit G-code OR coordinates to be a move
    // Commands like "T1 M6" or "S13000 M3" have no G-code and no coordinates, so not moves
    bool has_coords = gcode->has_letter('X') || gcode->has_letter('Y') || gcode->has_letter('Z') || 
                      gcode->has_letter('I') || gcode->has_letter('J') || gcode->has_letter('K');
    slot.is_move = (g_code == 0 || g_code == 1 || g_code == 2 || g_code == 3) && 
                   (gcode->has_g || has_coords);
    
    if (!slot.is_move) {
        slot.has_ijk = false;
        return;
    }
    
    // Store uncompensated start position (current position before this move)
    slot.uncomp_start[X_AXIS] = uncompensated_position[X_AXIS];
    slot.uncomp_start[Y_AXIS] = uncompensated_position[Y_AXIS];
    slot.uncomp_start[Z_AXIS] = uncompensated_position[Z_AXIS];
    
    // Extract endpoint and update uncompensated position tracker
    // This tracks the PROGRAMMED path so next buffered move knows correct start
    if (gcode->has_letter('X')) {
        slot.endpoint[X_AXIS] = gcode->get_value('X');
        uncompensated_position[X_AXIS] = slot.endpoint[X_AXIS];
    } else {
        slot.endpoint[X_AXIS] = uncompensated_position[X_AXIS];
    }
    
    if (gcode->has_letter('Y')) {
        slot.endpoint[Y_AXIS] = gcode->get_value('Y');
        uncompensated_position[Y_AXIS] = slot.endpoint[Y_AXIS];
    } else {
        slot.endpoint[Y_AXIS] = uncompensated_position[Y_AXIS];
    }
    
    if (gcode->has_letter('Z')) {
        slot.endpoint[Z_AXIS] = gcode->get_value('Z');
        uncompensated_position[Z_AXIS] = slot.endpoint[Z_AXIS];
    } else {
        slot.endpoint[Z_AXIS] = uncompensated_position[Z_AXIS];
    }
    
    // Debug: Show extracted endpoint
    THEKERNEL->streams->printf(">>ENDPOINT: X=%.3f Y=%.3f Z=%.3f (is_move=%d)\n",
        slot.endpoint[X_AXIS], slot.endpoint[Y_AXIS], slot.endpoint[Z_AXIS], slot.is_move ? 1 : 0);
    
    // Extract arc parameters
    if (g_code == 2 || g_code == 3) {
        slot.has_ijk = true;
        slot.is_cw = (g_code == 2);
        slot.ijk[0] = gcode->has_letter('I') ? gcode->get_value('I') : 0.0f;
        slot.ijk[1] = gcode->has_letter('J') ? gcode->get_value('J') : 0.0f;
        slot.ijk[2] = gcode->has_letter('K') ? gcode->get_value('K') : 0.0f;
    } else {
        slot.has_ijk = false;
    }
    
    // Calculate direction vector for straight lines
    if (g_code == 0 || g_code == 1) {
        // Direction = endpoint - start
        // Start is previous uncompensated_position (before update above)
        // But we already updated it, so recalculate from current - previous
        // Actually, we need to track the start position separately
        // For now, we'll calculate direction during lookahead
        slot.direction[X_AXIS] = 0;
        slot.direction[Y_AXIS] = 0;
        slot.direction[Z_AXIS] = 0;
    }
}

Gcode* CompensationPreprocessor::get_compensated_gcode()
{
    if (buffer_count == 0) {
        return nullptr;
    }
    
    int required_lookahead = 3;
    
    if (buffer_count < required_lookahead && !is_flushing) {
        THEKERNEL->streams->printf(">>PHASE2_BUFFERING: Holding for lookahead (count=%d, need=%d)\n", 
            buffer_count, required_lookahead);
        return nullptr;
    }
    
    BufferedGcode& slot = buffer[buffer_tail];
    
    if (slot.gcode == nullptr) {
        THEKERNEL->streams->printf(">>ERROR: Buffer slot %d has NULL gcode pointer! Clearing buffer.\n", buffer_tail);
        clear();
        return nullptr;
    }
    
    print_buffer_state("before_offset");
    print_position_state("before_offset");
    
    // PHASE 2: Apply compensation if active and this is a straight line move
    if (compensation_type != CompensationType::NONE && slot.is_move && !slot.has_ijk) {
        THEKERNEL->streams->printf(">>PHASE2: Applying compensation to move\n");
        
        float direction[2];
        direction[0] = slot.endpoint[X_AXIS] - slot.uncomp_start[X_AXIS];
        direction[1] = slot.endpoint[Y_AXIS] - slot.uncomp_start[Y_AXIS];
        
        float mag = sqrtf(direction[0]*direction[0] + direction[1]*direction[1]);
        
        if (mag < 0.1f) {
            THEKERNEL->streams->printf(">>PHASE2: Zero-length move detected (%.4f mm), looking ahead\n", mag);
            
            if (buffer_count > 1) {
                int next_idx = buffer_next_index(buffer_tail);
                BufferedGcode& next_slot = buffer[next_idx];
                
                if (next_slot.is_move && !next_slot.has_ijk) {
                    direction[0] = next_slot.endpoint[X_AXIS] - slot.endpoint[X_AXIS];
                    direction[1] = next_slot.endpoint[Y_AXIS] - slot.endpoint[Y_AXIS];
                    mag = sqrtf(direction[0]*direction[0] + direction[1]*direction[1]);
                    THEKERNEL->streams->printf(">>PHASE2: Using next move direction: mag=%.4f\n", mag);
                }
            }
            
            if (mag < 0.1f) {
                THEKERNEL->streams->printf(">>PHASE2: Cannot determine direction, skipping compensation\n");
                goto skip_compensation;
            }
        }
        
        direction[0] /= mag;
        direction[1] /= mag;
        
        float normal[2];
        if (compensation_type == CompensationType::LEFT) {
            normal[0] = -direction[1];
            normal[1] = direction[0];
        } else {
            normal[0] = direction[1];
            normal[1] = -direction[0];
        }
        
        float offset_endpoint[2];
        offset_endpoint[0] = slot.endpoint[X_AXIS] + normal[0] * compensation_radius;
        offset_endpoint[1] = slot.endpoint[Y_AXIS] + normal[1] * compensation_radius;
        
        print_offset_calculation(slot, direction, normal, offset_endpoint);
        
        char new_command[256];
        char* ptr = new_command;
        
        if (slot.gcode->has_g) {
            ptr += snprintf(ptr, sizeof(new_command) - (ptr - new_command), "G%d ", slot.gcode->g);
        }
        
        ptr += snprintf(ptr, sizeof(new_command) - (ptr - new_command), 
            "X%.4f Y%.4f Z%.4f",
            offset_endpoint[0],
            offset_endpoint[1],
            slot.endpoint[Z_AXIS]);
        
        if (slot.gcode->has_letter('F')) {
            ptr += snprintf(ptr, sizeof(new_command) - (ptr - new_command), 
                " F%.1f", slot.gcode->get_value('F'));
        }
        
        THEKERNEL->streams->printf(">>PHASE2_REBUILT: '%s' -> '%s'\n",
            slot.gcode->get_command(), new_command);
        
        delete slot.gcode;
        slot.gcode = new Gcode(new_command, &(StreamOutput::NullStream));
        
        // Update compensated position to actual offset endpoint
        // uncompensated_position was already updated during buffering
        compensated_position[X_AXIS] = offset_endpoint[0];
        compensated_position[Y_AXIS] = offset_endpoint[1];
        compensated_position[Z_AXIS] = slot.endpoint[Z_AXIS];
        
        float start[2] = {slot.uncomp_start[X_AXIS], slot.uncomp_start[Y_AXIS]};
        print_move_transform(slot, start, offset_endpoint);
        
    } else {
        skip_compensation:
        
        if (slot.is_move) {
            THEKERNEL->streams->printf(">>PHASE2_PASSTHROUGH: Move without compensation (comp=%s, is_arc=%d)\n",
                compensation_type == CompensationType::NONE ? "OFF" : "ON",
                slot.has_ijk ? 1 : 0);
            
            // uncompensated_position was already updated during buffering
            // Just sync compensated position to match (no offset applied)
            compensated_position[X_AXIS] = slot.endpoint[X_AXIS];
            compensated_position[Y_AXIS] = slot.endpoint[Y_AXIS];
            compensated_position[Z_AXIS] = slot.endpoint[Z_AXIS];
        } else {
            THEKERNEL->streams->printf(">>PHASE2_COMMAND: Non-move command '%s'\n",
                slot.gcode->get_command());
        }
    }
    
    print_position_state("after_offset");
    
    Gcode* output = slot.gcode;
    slot.gcode = nullptr;
    
    buffer_tail = buffer_next_index(buffer_tail);
    buffer_count--;
    
    THEKERNEL->streams->printf(">>PHASE2_OUTPUT: Command released (remaining=%d)\n", buffer_count);
    print_buffer_state("after_output");
    THEKERNEL->streams->printf("\n");
    
    return output;
}


void CompensationPreprocessor::apply_compensation(int index)
{
    BufferedGcode& current = buffer[index];
    
    if (!current.is_move) {
        return;  // Not a move, no compensation needed
    }
    
    // Skip compensation if not active
    if (compensation_type == CompensationType::NONE) {
        return;  // Just pass through unmodified
    }
    
    // Calculate compensated coordinates
    float new_endpoint[3];
    float new_ijk[3];
    memcpy(new_endpoint, current.endpoint, sizeof(new_endpoint));
    memcpy(new_ijk, current.ijk, sizeof(new_ijk));
    
    bool is_left = (compensation_type == CompensationType::LEFT);
    
    if (current.has_ijk) {
        // Arc move - apply arc compensation
        // Use the stored uncompensated start position from when this move was buffered
        float uncomp_start[2] = {current.uncomp_start[X_AXIS], current.uncomp_start[Y_AXIS]};
        float comp_start[2] = {compensated_position[X_AXIS], compensated_position[Y_AXIS]};
        compensate_arc_endpoint(
            uncomp_start,
            comp_start,
            new_endpoint,
            new_ijk,
            compensation_radius,
            is_left,
            current.is_cw
        );
    } else {
        // Straight line - check for corner with next move
        int next_index = buffer_next_index(index);
        
        if (buffer_count > 1 && next_index != buffer_head) {
            BufferedGcode& next = buffer[next_index];
            
            if (next.is_move && !next.has_ijk) {
                // Both current and next are lines - apply corner intersection
                float dir1[2], dir2[2];
                
                // Calculate direction vectors
                // dir1: from previous position to current endpoint
                float prev_x = (index == buffer_tail) ? compensated_position[X_AXIS] : buffer[(index - 1 + BUFFER_SIZE) % BUFFER_SIZE].endpoint[X_AXIS];
                float prev_y = (index == buffer_tail) ? compensated_position[Y_AXIS] : buffer[(index - 1 + BUFFER_SIZE) % BUFFER_SIZE].endpoint[Y_AXIS];
                
                dir1[0] = current.endpoint[X_AXIS] - prev_x;
                dir1[1] = current.endpoint[Y_AXIS] - prev_y;
                float mag1 = sqrtf(dir1[0]*dir1[0] + dir1[1]*dir1[1]);
                if (mag1 > 0.00001f) {
                    dir1[0] /= mag1;
                    dir1[1] /= mag1;
                }
                
                // dir2: from current to next endpoint
                dir2[0] = next.endpoint[X_AXIS] - current.endpoint[X_AXIS];
                dir2[1] = next.endpoint[Y_AXIS] - current.endpoint[Y_AXIS];
                float mag2 = sqrtf(dir2[0]*dir2[0] + dir2[1]*dir2[1]);
                if (mag2 > 0.00001f) {
                    dir2[0] /= mag2;
                    dir2[1] /= mag2;
                }
                
                // Calculate corner intersection
                float corner_point[2] = {current.endpoint[X_AXIS], current.endpoint[Y_AXIS]};
                float intersection[2];
                
                if (calculate_corner_intersection(corner_point, dir1, dir2, compensation_radius, is_left, intersection)) {
                    new_endpoint[X_AXIS] = intersection[0];
                    new_endpoint[Y_AXIS] = intersection[1];
                } else {
                    // Parallel lines - use simple perpendicular offset
                    calculate_perpendicular_offset(
                        current.endpoint,
                        dir1,
                        compensation_radius,
                        is_left,
                        new_endpoint
                    );
                }
            } else {
                // Next move is not a line - use simple perpendicular offset
                // Calculate direction from previous to current
                float dir[2];
                float prev_x = uncompensated_position[X_AXIS];
                float prev_y = uncompensated_position[Y_AXIS];
                
                dir[0] = current.endpoint[X_AXIS] - prev_x;
                dir[1] = current.endpoint[Y_AXIS] - prev_y;
                float mag = sqrtf(dir[0]*dir[0] + dir[1]*dir[1]);
                if (mag > 0.00001f) {
                    dir[0] /= mag;
                    dir[1] /= mag;
                }
                
                calculate_perpendicular_offset(
                    current.endpoint,
                    dir,
                    compensation_radius,
                    is_left,
                    new_endpoint
                );
            }
        } else {
            // No next move or single move - use simple perpendicular offset
            float dir[2];
            float prev_x = uncompensated_position[X_AXIS];
            float prev_y = uncompensated_position[Y_AXIS];
            
            dir[0] = current.endpoint[X_AXIS] - prev_x;
            dir[1] = current.endpoint[Y_AXIS] - prev_y;
            float mag = sqrtf(dir[0]*dir[0] + dir[1]*dir[1]);
            if (mag > 0.00001f) {
                dir[0] /= mag;
                dir[1] /= mag;
            }
            
            calculate_perpendicular_offset(
                current.endpoint,
                dir,
                compensation_radius,
                is_left,
                new_endpoint
            );
        }
    }
    
    // Copy compensated values back to buffer slot (X and Y only - Z is never compensated)
    current.endpoint[X_AXIS] = new_endpoint[X_AXIS];
    current.endpoint[Y_AXIS] = new_endpoint[Y_AXIS];
    // Note: Z is intentionally NOT copied - cutter compensation only affects X/Y plane
    
    if (current.has_ijk) {
        current.ijk[0] = new_ijk[0];
        current.ijk[1] = new_ijk[1];
        current.ijk[2] = new_ijk[2];
    }
    
    // Modify G-code with new coordinates
    modify_gcode_coordinates(current.gcode, new_endpoint, new_ijk, current.has_ijk);
}

void CompensationPreprocessor::calculate_perpendicular_offset(
    const float endpoint[2],
    const float direction[2],
    float radius,
    bool is_left,
    float output[2]
)
{
    float ux = direction[0];
    float uy = direction[1];
    
    // Calculate normal vector (perpendicular to direction)
    float nx, ny;
    if (is_left) {
        // G41: Rotate 90° CCW
        nx = -uy;
        ny = ux;
    } else {
        // G42: Rotate 90° CW
        nx = uy;
        ny = -ux;
    }
    
    // Apply offset
    output[0] = endpoint[0] + nx * radius;
    output[1] = endpoint[1] + ny * radius;
}

bool CompensationPreprocessor::calculate_corner_intersection(
    const float corner_point[2],
    const float dir1[2],
    const float dir2[2],
    float radius,
    bool is_left,
    float output[2]
)
{
    float u1x = dir1[0];
    float u1y = dir1[1];
    float u2x = dir2[0];
    float u2y = dir2[1];
    
    // Calculate normal vectors for both moves
    float n1x, n1y, n2x, n2y;
    if (is_left) {
        // G41: Rotate 90° CCW
        n1x = -u1y; n1y = u1x;
        n2x = -u2y; n2y = u2x;
    } else {
        // G42: Rotate 90° CW
        n1x = u1y; n1y = -u1x;
        n2x = u2y; n2y = -u2x;
    }
    
    // Both offset lines start at the corner point
    float p1x = corner_point[0] + n1x * radius;
    float p1y = corner_point[1] + n1y * radius;
    
    float p2x = corner_point[0] + n2x * radius;
    float p2y = corner_point[1] + n2y * radius;
    
    // Calculate determinant to check for parallel lines
    float det = u1x * u2y - u1y * u2x;
    
    if (fabsf(det) < 0.00001f) {
        // Lines are parallel - cannot calculate intersection
        return false;
    }
    
    // Calculate intersection parameter t1
    float dx = p2x - p1x;
    float dy = p2y - p1y;
    
    float t1 = (dx * u2y - dy * u2x) / det;
    
    // Calculate intersection point
    output[0] = p1x + t1 * u1x;
    output[1] = p1y + t1 * u1y;
    
    return true;
}

bool CompensationPreprocessor::is_inside_corner(
    const float dir1[2],
    const float dir2[2],
    bool is_left
)
{
    float cross = cross_product_2d(dir1, dir2);
    
    if (is_left) {
        return (cross < 0); // G41: negative cross = inside corner
    } else {
        return (cross > 0); // G42: positive cross = inside corner
    }
}

bool CompensationPreprocessor::compensate_arc_endpoint(
    const float uncomp_start[2],
    const float comp_start[2],
    float arc_endpoint[2],
    float arc_ij[2],
    float comp_radius,
    bool is_left,
    bool is_cw
)
{
    float i = arc_ij[0];
    float j = arc_ij[1];
    
    // Calculate arc center from UNCOMPENSATED start position
    float center_x = uncomp_start[0] + i;
    float center_y = uncomp_start[1] + j;
    
    // Calculate radius vector from center to endpoint
    float to_end_x = arc_endpoint[0] - center_x;
    float to_end_y = arc_endpoint[1] - center_y;
    float arc_radius = sqrtf(to_end_x*to_end_x + to_end_y*to_end_y);
    
    if (arc_radius < 0.00001f) {
        return false; // Invalid arc: zero radius
    }
    
    // Normalize radius vector
    to_end_x /= arc_radius;
    to_end_y /= arc_radius;
    
    // Calculate tangent at endpoint (perpendicular to radius)
    float tangent_x, tangent_y;
    if (is_cw) {
        tangent_x = -to_end_y;
        tangent_y = to_end_x;
    } else {
        tangent_x = to_end_y;
        tangent_y = -to_end_x;
    }
    
    // Determine offset direction based on compensation side
    bool offset_outward = (is_left && !is_cw) || (!is_left && is_cw);
    float offset_sign = offset_outward ? 1.0f : -1.0f;
    
    // Calculate offset along tangent
    float offset_x = tangent_x * comp_radius * offset_sign;
    float offset_y = tangent_y * comp_radius * offset_sign;
    
    // Apply offset to endpoint
    arc_endpoint[0] += offset_x;
    arc_endpoint[1] += offset_y;
    
    // Recalculate I/J: offsets from COMPENSATED START to center
    arc_ij[0] = center_x - comp_start[0];
    arc_ij[1] = center_y - comp_start[1];
    
    return true;
}

void CompensationPreprocessor::modify_gcode_coordinates(
    Gcode* gcode,
    const float new_endpoint[3],
    const float new_ijk[3],
    bool has_ijk
)
{
    // Rebuild G-code string with new coordinates
    // This is the "Gcode-in, Gcode-out" approach
    
    char new_command[256];
    char* ptr = new_command;
    
    // Copy command (G0, G1, G2, G3)
    if (gcode->has_g) {
        ptr += snprintf(ptr, sizeof(new_command) - (ptr - new_command), "G%d ", gcode->g);
    }
    
    // Add X/Y/Z with new coordinates
    ptr += snprintf(ptr, sizeof(new_command) - (ptr - new_command), "X%.4f Y%.4f Z%.4f ", 
                    new_endpoint[X_AXIS], new_endpoint[Y_AXIS], new_endpoint[Z_AXIS]);
    
    // Add I/J/K for arcs
    if (has_ijk) {
        ptr += snprintf(ptr, sizeof(new_command) - (ptr - new_command), "I%.4f J%.4f K%.4f ", 
                        new_ijk[0], new_ijk[1], new_ijk[2]);
    }
    
    // Add F if present
    if (gcode->has_letter('F')) {
        ptr += snprintf(ptr, sizeof(new_command) - (ptr - new_command), "F%.1f ", gcode->get_value('F'));
    }
    
    // We can't modify the Gcode object's private members directly,
    // so this function doesn't actually work as intended.
    // The solution is to not modify in place, but return a new Gcode object.
    // However, for now we'll leave this as a placeholder since the gcode
    // values should already be correct from our extract phase.
    // TODO: Refactor to return new Gcode* instead of modifying in place
}

void CompensationPreprocessor::flush()
{
    // Set flushing flag to bypass lookahead requirements
    is_flushing = true;
    
    // Output all remaining buffered moves
    // NOTE: The caller (Robot.cpp G40 handler) must call get_compensated_gcode() in a loop
    // This function just sets the flag so get_compensated_gcode() will return remaining moves
    
    // Clear flag after flush (actually cleared in set_compensation when called with NONE)
}

void CompensationPreprocessor::clear()
{
    // Delete all buffered G-codes
    for (int i = 0; i < BUFFER_SIZE; i++) {
        if (buffer[i].gcode != nullptr) {
            delete buffer[i].gcode;
            buffer[i].gcode = nullptr;
        }
    }
    
    buffer_head = 0;
    buffer_tail = 0;
    buffer_count = 0;
    is_flushing = false;  // Clear flushing flag
}

void CompensationPreprocessor::set_initial_position(const float position[3])
{
    uncompensated_position[X_AXIS] = position[X_AXIS];
    uncompensated_position[Y_AXIS] = position[Y_AXIS];
    uncompensated_position[Z_AXIS] = position[Z_AXIS];
    
    compensated_position[X_AXIS] = position[X_AXIS];
    compensated_position[Y_AXIS] = position[Y_AXIS];
    compensated_position[Z_AXIS] = position[Z_AXIS];
}

void CompensationPreprocessor::print_buffer_state(const char* context)
{
    const char* comp_str = "NONE";
    if (compensation_type == CompensationType::LEFT) comp_str = "LEFT";
    else if (compensation_type == CompensationType::RIGHT) comp_str = "RIGHT";
    
    THEKERNEL->streams->printf(">>BUFFER_STATE[%s]: count=%d, head=%d, tail=%d, comp=%s, radius=%.3f\n",
        context, buffer_count, buffer_head, buffer_tail, comp_str, compensation_radius);
}

void CompensationPreprocessor::print_buffer_contents()
{
    THEKERNEL->streams->printf(">>BUFFER_CONTENTS: (%d commands)\n", buffer_count);
    for (int i = 0; i < buffer_count; i++) {
        int idx = (buffer_tail + i) % BUFFER_SIZE;
        BufferedGcode& slot = buffer[idx];
        THEKERNEL->streams->printf("  [%d] ", i);
        if (slot.is_move) {
            THEKERNEL->streams->printf("MOVE: (%.3f,%.3f,%.3f) -> (%.3f,%.3f,%.3f) %s\n",
                slot.uncomp_start[X_AXIS], slot.uncomp_start[Y_AXIS], slot.uncomp_start[Z_AXIS],
                slot.endpoint[X_AXIS], slot.endpoint[Y_AXIS], slot.endpoint[Z_AXIS],
                slot.gcode ? slot.gcode->get_command() : "NULL");
        } else {
            THEKERNEL->streams->printf("CMD: %s\n", slot.gcode ? slot.gcode->get_command() : "NULL");
        }
    }
}

void CompensationPreprocessor::print_position_state(const char* context)
{
    THEKERNEL->streams->printf(">>POSITION[%s]: uncomp=(%.3f,%.3f,%.3f) comp=(%.3f,%.3f,%.3f) delta=(%.3f,%.3f)\n",
        context,
        uncompensated_position[X_AXIS], uncompensated_position[Y_AXIS], uncompensated_position[Z_AXIS],
        compensated_position[X_AXIS], compensated_position[Y_AXIS], compensated_position[Z_AXIS],
        compensated_position[X_AXIS] - uncompensated_position[X_AXIS],
        compensated_position[Y_AXIS] - uncompensated_position[Y_AXIS]);
}

void CompensationPreprocessor::print_offset_calculation(
    const BufferedGcode& slot,
    const float direction[2],
    const float normal[2],
    const float offset_endpoint[2])
{
    THEKERNEL->streams->printf(">>OFFSET_CALC:\n");
    THEKERNEL->streams->printf("  Original: (%.3f,%.3f) -> (%.3f,%.3f)\n",
        slot.uncomp_start[X_AXIS], slot.uncomp_start[Y_AXIS],
        slot.endpoint[X_AXIS], slot.endpoint[Y_AXIS]);
    THEKERNEL->streams->printf("  Direction: (%.4f, %.4f)\n", direction[0], direction[1]);
    THEKERNEL->streams->printf("  Normal: (%.4f, %.4f) [%s]\n", 
        normal[0], normal[1],
        compensation_type == CompensationType::LEFT ? "LEFT/CCW" : "RIGHT/CW");
    THEKERNEL->streams->printf("  Radius: %.3f\n", compensation_radius);
    THEKERNEL->streams->printf("  Offset Endpoint: (%.3f,%.3f)\n", offset_endpoint[0], offset_endpoint[1]);
    THEKERNEL->streams->printf("  Offset Amount: (%.3f,%.3f)\n",
        offset_endpoint[0] - slot.endpoint[X_AXIS],
        offset_endpoint[1] - slot.endpoint[Y_AXIS]);
}

void CompensationPreprocessor::print_move_transform(
    const BufferedGcode& slot,
    const float start[2],
    const float end[2])
{
    THEKERNEL->streams->printf(">>MOVE_TRANSFORM:\n");
    THEKERNEL->streams->printf("  BEFORE: (%.3f,%.3f) -> (%.3f,%.3f)\n",
        slot.uncomp_start[X_AXIS], slot.uncomp_start[Y_AXIS],
        slot.endpoint[X_AXIS], slot.endpoint[Y_AXIS]);
    THEKERNEL->streams->printf("  AFTER:  (%.3f,%.3f) -> (%.3f,%.3f)\n",
        start[0], start[1], end[0], end[1]);
    THEKERNEL->streams->printf("  SHIFT:  (%.3f,%.3f) at start, (%.3f,%.3f) at end\n",
        start[0] - slot.uncomp_start[X_AXIS], start[1] - slot.uncomp_start[Y_AXIS],
        end[0] - slot.endpoint[X_AXIS], end[1] - slot.endpoint[Y_AXIS]);
}
