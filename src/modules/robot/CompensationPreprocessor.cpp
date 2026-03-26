/*
      This file is part of Smoothie (http://smoothieware.org/).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Gcode.h"
#include "CompensationPreprocessor.h"
#include "mri.h"
#include "nuts_bolts.h"
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "Conveyor.h"
#include "libs/StreamOutput.h"
#include "StreamOutputPool.h"

#include <cstring>
#include <cmath>
#include <cstdio>

using namespace std;

CompensationPreprocessor::CompensationPreprocessor()
{
    // Initialize uncomp buffer
    uncomp_head = 0;
    uncomp_tail = 0;
    uncomp_count = 0;
    
    // Initialize comp buffer
    comp_head = 0;
    comp_tail = 0;
    comp_count = 0;
    comp_ready_count = 0;
    
    // Initialize compensation state
    comp_type = CompensationType::NONE;
    comp_radius = 0.0f;
    comp_active = false;
    is_flushing = false;
    pipeline_primed = false;
    first_output_pending = true;
    has_initial_position = false;
    last_g = 0;  // Default to G0
    initial_position[0] = 0.0f;
    initial_position[1] = 0.0f;
    initial_position[2] = 0.0f;
    
    // Initialize buffer memory to zero
    for (int i = 0; i < BUFFER_SIZE; i++) {
        uncomp_ring[i].x = 0.0f;
        uncomp_ring[i].y = 0.0f;
        uncomp_ring[i].z = 0.0f;
        comp_ring[i].x = 0.0f;
        comp_ring[i].y = 0.0f;
        comp_ring[i].z = 0.0f;
        comp_ring[i].gcode = nullptr;  // Initialize Gcode pointers
    }
}

CompensationPreprocessor::~CompensationPreprocessor()
{
    clear();
}

void CompensationPreprocessor::set_compensation(CompensationType type, float radius)
{
    THEKERNEL->streams->printf(">>SET_COMP: ENTRY - type=%d, radius=%.3f\n", (int)type, radius);
    
    comp_type = type;
    comp_radius = radius;
    
    if (type == CompensationType::NONE) {
        // G40 - Turn off compensation after explicit flush has drained buffers
        is_flushing = false;
        comp_active = false;
        pipeline_primed = false;
        first_output_pending = true;
        has_initial_position = false;
        THEKERNEL->streams->printf(">>PHASE2: G40 activated - compensation OFF\n");
    } else {
        // G41/G42 - Turn on compensation
        THEKERNEL->streams->printf(">>PHASE2: G41/G42 called - clearing buffer (count was %d)\n", uncomp_count);
        clear();
        THEKERNEL->streams->printf(">>SET_COMP: clear() returned successfully\n");
        
        is_flushing = false;
        comp_active = true;
        pipeline_primed = false;
        first_output_pending = true;
        has_initial_position = false;
        
        const char* side_str = (type == CompensationType::LEFT) ? "LEFT (G41)" : "RIGHT (G42)";
        THEKERNEL->streams->printf(">>PHASE2: Compensation activated - type=%s, radius=%.3f\n", side_str, radius);
    }
    
    THEKERNEL->streams->printf(">>SET_COMP: EXIT - comp_active=%d\n", comp_active);
}

Gcode* CompensationPreprocessor::buffer_gcode(Gcode* gcode)
{
    THEKERNEL->streams->printf(">>BUFFER_GCODE: ENTRY - gcode=%p, uncomp_count=%d, comp_ready=%d\n",
        (void*)gcode, uncomp_count, comp_ready_count);

    if (gcode == nullptr) {
        THEKERNEL->streams->printf(">>BUFFER_GCODE: Null gcode pointer\n");
        return nullptr;
    }
    
    // Extract coordinates (modal if not specified)
    float x, y, z;
    
    if (uncomp_count == 0) {
        // First point - use explicit coordinates or captured activation position
        THEKERNEL->streams->printf(">>BUFFER_GCODE: First move after compensation activation\n");
        x = gcode->has_letter('X') ? gcode->get_value('X') : initial_position[X_AXIS];
        y = gcode->has_letter('Y') ? gcode->get_value('Y') : initial_position[Y_AXIS];
        z = gcode->has_letter('Z') ? gcode->get_value('Z') : initial_position[Z_AXIS];
        THEKERNEL->streams->printf(">>BUFFER_GCODE: Coords X=%.3f Y=%.3f Z=%.3f\n", x, y, z);
    } else {
        // Use modal coordinates (last position if not specified)
        int prev_idx = (uncomp_head - 1 + BUFFER_SIZE) % BUFFER_SIZE;
        x = gcode->has_letter('X') ? gcode->get_value('X') : uncomp_ring[prev_idx].x;
        y = gcode->has_letter('Y') ? gcode->get_value('Y') : uncomp_ring[prev_idx].y;
        z = gcode->has_letter('Z') ? gcode->get_value('Z') : uncomp_ring[prev_idx].z;
    }
    
    // Store in uncomp buffer
    uncomp_ring[uncomp_head].x = x;
    uncomp_ring[uncomp_head].y = y;
    uncomp_ring[uncomp_head].z = z;
    
    uncomp_head = (uncomp_head + 1) % BUFFER_SIZE;
    if (uncomp_count < BUFFER_SIZE) {
        uncomp_count++;
    }
    
    print_uncomp_buffered();

    if (uncomp_count < BUFFER_SIZE) {
        THEKERNEL->streams->printf(">>BUFFER_GCODE: Priming uncomp buffer (count=%d)\n", uncomp_count);
        return nullptr;
    }

    if (!pipeline_primed) {
        THEKERNEL->streams->printf(">>BUFFER_GCODE: First full cycle - priming with double compute\n");
        compute_and_output();
        if (comp_ready_count < BUFFER_SIZE) {
            compute_and_output();
        }
        pipeline_primed = true;
        print_buffer_state();
        return nullptr;
    }
    
    if (comp_ready_count < BUFFER_SIZE) {
        THEKERNEL->streams->printf(">>BUFFER_GCODE: Steady-state single compute\n");
        compute_and_output();
    }

    Gcode* output = serve_compensated_gcode(false);
    THEKERNEL->streams->printf(">>BUFFER_GCODE: EXIT - returning %p\n", (void*)output);
    return output;
}

// ============================================================================
// PHASE-LOCKED DUAL BUFFER PROCESSING
// ============================================================================

void CompensationPreprocessor::compute_and_output()
{
    THEKERNEL->streams->printf(">>COMPUTE: ENTRY - uncomp_count=%d, comp_count=%d, comp_ready=%d\n",
        uncomp_count, comp_count, comp_ready_count);

    if (uncomp_count < 2) {
        THEKERNEL->streams->printf(">>COMPUTE: Not enough uncomp points to compute\n");
        return;
    }

    if (comp_ready_count >= BUFFER_SIZE) {
        THEKERNEL->streams->printf(">>COMPUTE: Comp ring has no free slot, skip\n");
        return;
    }

    int comp_idx = comp_head;
    if (comp_ring[comp_idx].gcode != nullptr) {
        THEKERNEL->streams->printf(">>COMPUTE: Target comp slot %d still occupied, skip\n", comp_idx);
        return;
    }
    
    // Calculate offset based on whether this is the first point or subsequent
    float offset[2];
    bool success;
    
    if (first_output_pending) {
        // ====================================================================
        // SPECIAL CASE: First point after G41/G42 activation
        // Use ONSET direction (current → next) to determine offset
        // ====================================================================
        
        int uncomp_curr = uncomp_tail;
        int uncomp_next = (uncomp_tail + 1) % BUFFER_SIZE;
        const UncompPoint& curr = uncomp_ring[uncomp_curr];
        const UncompPoint& next = uncomp_ring[uncomp_next];
        
        THEKERNEL->streams->printf(">>COMPUTE: FIRST POINT - using onset direction curr[%d]→next[%d]\n", 
            uncomp_curr, uncomp_next);
        THEKERNEL->streams->printf("  curr=(%.3f,%.3f,%.3f), next=(%.3f,%.3f,%.3f)\n",
            curr.x, curr.y, curr.z, next.x, next.y, next.z);
        
        // Calculate offset using onset direction
        success = calculate_perpendicular_offset(curr, next, offset);
        
        if (success) {
            comp_ring[comp_idx].x = offset[0];
            comp_ring[comp_idx].y = offset[1];
            comp_ring[comp_idx].z = curr.z;
            THEKERNEL->streams->printf(">>COMPUTE: First point offset - comp[%d]=(%.3f,%.3f,%.3f)\n",
                comp_idx, offset[0], offset[1], curr.z);
        } else {
            // Zero-length move: use uncompensated coordinates
            comp_ring[comp_idx].x = curr.x;
            comp_ring[comp_idx].y = curr.y;
            comp_ring[comp_idx].z = curr.z;
            THEKERNEL->streams->printf(">>COMPUTE: First point - zero-length, no offset\n");
        }
        
        // Build Gcode string
        char gcode_str[128];
        snprintf(gcode_str, sizeof(gcode_str), "G1 X%.3f Y%.3f Z%.3f",
                 comp_ring[comp_idx].x, comp_ring[comp_idx].y, comp_ring[comp_idx].z);
        
        print_output(gcode_str);
        
        // Create and store Gcode object
        THEKERNEL->streams->printf(">>COMPUTE: Creating first Gcode at comp[%d]\n", comp_idx);
        comp_ring[comp_idx].gcode = new Gcode(gcode_str, &StreamOutput::NullStream);
        THEKERNEL->streams->printf(">>COMPUTE: Gcode created at %p\n", 
            (void*)comp_ring[comp_idx].gcode);
        
        if (comp_count < BUFFER_SIZE) {
            comp_count++;
        }
        comp_ready_count++;
        uncomp_tail = (uncomp_tail + 1) % BUFFER_SIZE;
        comp_head = (comp_head + 1) % BUFFER_SIZE;
        first_output_pending = false;

        THEKERNEL->streams->printf(">>COMPUTE: EXIT - first point complete, comp_count=%d, comp_ready=%d, uncomp_tail=%d, comp_head=%d\n",
            comp_count, comp_ready_count, uncomp_tail, comp_head);
        
    } else {
        // ====================================================================
        // NORMAL CASE: Subsequent points (phase-locked processing)
        // Use ENTRY direction (prev → current) to determine offset
        // ====================================================================
        
        int uncomp_prev = uncomp_tail;
        int uncomp_curr = (uncomp_tail + 1) % BUFFER_SIZE;
        const UncompPoint& prev = uncomp_ring[uncomp_prev];
        const UncompPoint& curr = uncomp_ring[uncomp_curr];
        
        THEKERNEL->streams->printf(">>COMPUTE: SUBSEQUENT POINT - using entry direction prev[%d]→curr[%d]\n",
            uncomp_prev, uncomp_curr);
        THEKERNEL->streams->printf("  prev=(%.3f,%.3f,%.3f), curr=(%.3f,%.3f,%.3f)\n",
            prev.x, prev.y, prev.z, curr.x, curr.y, curr.z);
        
        // Calculate offset using entry direction
        success = calculate_perpendicular_offset(prev, curr, offset);
        
        if (success) {
            comp_ring[comp_idx].x = offset[0];
            comp_ring[comp_idx].y = offset[1];
            comp_ring[comp_idx].z = prev.z;
            THEKERNEL->streams->printf(">>COMPUTE: Subsequent offset - comp[%d]=(%.3f,%.3f,%.3f)\n",
                comp_idx, offset[0], offset[1], prev.z);
        } else {
            // Zero-length move: use uncompensated coordinates
            comp_ring[comp_idx].x = prev.x;
            comp_ring[comp_idx].y = prev.y;
            comp_ring[comp_idx].z = prev.z;
            THEKERNEL->streams->printf(">>COMPUTE: Subsequent - zero-length, no offset\n");
        }
        
        // Build Gcode string
        char gcode_str[128];
        snprintf(gcode_str, sizeof(gcode_str), "G1 X%.3f Y%.3f Z%.3f",
                 comp_ring[comp_idx].x, comp_ring[comp_idx].y, comp_ring[comp_idx].z);
        
        print_output(gcode_str);
        
        // Create and store Gcode object
        THEKERNEL->streams->printf(">>COMPUTE: Creating subsequent Gcode at comp[%d]\n", comp_idx);
        comp_ring[comp_idx].gcode = new Gcode(gcode_str, &StreamOutput::NullStream);
        THEKERNEL->streams->printf(">>COMPUTE: Gcode created at %p\n", 
            (void*)comp_ring[comp_idx].gcode);
        
        if (comp_count < BUFFER_SIZE) {
            comp_count++;
        }
        comp_ready_count++;
        uncomp_tail = (uncomp_tail + 1) % BUFFER_SIZE;
        comp_head = (comp_head + 1) % BUFFER_SIZE;

        THEKERNEL->streams->printf(">>COMPUTE: EXIT - subsequent complete, comp_count=%d, comp_ready=%d, uncomp_tail=%d, comp_head=%d\n",
            comp_count, comp_ready_count, uncomp_tail, comp_head);
    }
}

bool CompensationPreprocessor::compute_terminal_output()
{
    if (uncomp_count < 2 || comp_ready_count >= BUFFER_SIZE) {
        return false;
    }

    int prev_idx = (uncomp_head - 2 + BUFFER_SIZE) % BUFFER_SIZE;
    int last_idx = (uncomp_head - 1 + BUFFER_SIZE) % BUFFER_SIZE;
    int comp_idx = comp_head;

    if (comp_ring[comp_idx].gcode != nullptr) {
        THEKERNEL->streams->printf(">>TERMINAL: Target comp slot %d still occupied, skip\n", comp_idx);
        return false;
    }

    const UncompPoint& prev = uncomp_ring[prev_idx];
    const UncompPoint& curr = uncomp_ring[last_idx];

    float dx = curr.x - prev.x;
    float dy = curr.y - prev.y;
    float mag = sqrtf(dx*dx + dy*dy);

    if (mag < 0.1f) {
        comp_ring[comp_idx].x = curr.x;
        comp_ring[comp_idx].y = curr.y;
        comp_ring[comp_idx].z = curr.z;
    } else {
        float dir_x = dx / mag;
        float dir_y = dy / mag;
        float normal_x;
        float normal_y;

        if (comp_type == CompensationType::LEFT) {
            normal_x = -dir_y;
            normal_y = dir_x;
        } else {
            normal_x = dir_y;
            normal_y = -dir_x;
        }

        comp_ring[comp_idx].x = curr.x + normal_x * comp_radius;
        comp_ring[comp_idx].y = curr.y + normal_y * comp_radius;
        comp_ring[comp_idx].z = curr.z;
    }

    char gcode_str[128];
    snprintf(gcode_str, sizeof(gcode_str), "G1 X%.3f Y%.3f Z%.3f",
             comp_ring[comp_idx].x, comp_ring[comp_idx].y, comp_ring[comp_idx].z);

    comp_ring[comp_idx].gcode = new Gcode(gcode_str, &StreamOutput::NullStream);

    comp_head = (comp_head + 1) % BUFFER_SIZE;
    comp_ready_count++;
    if (comp_count < BUFFER_SIZE) {
        comp_count++;
    }
    uncomp_tail = uncomp_head;

    THEKERNEL->streams->printf(">>TERMINAL: Added final point comp[%d], comp_ready=%d\n", comp_idx, comp_ready_count);
    return true;
}

bool CompensationPreprocessor::calculate_perpendicular_offset(
    const UncompPoint& prev,
    const UncompPoint& curr,
    float output[2])
{
    // Calculate direction vector: prev → curr
    float dx = curr.x - prev.x;
    float dy = curr.y - prev.y;
    float mag = sqrtf(dx*dx + dy*dy);
    
    if (mag < 0.1f) {
        // Zero-length move: cannot calculate direction
        return false;
    }
    
    // Normalize direction
    float dir[2];
    dir[0] = dx / mag;
    dir[1] = dy / mag;
    
    // Calculate perpendicular normal
    float normal[2];
    if (comp_type == CompensationType::LEFT) {
        // G41 LEFT: 90° CCW rotation
        normal[0] = -dir[1];
        normal[1] = dir[0];
    } else {
        // G42 RIGHT: 90° CW rotation
        normal[0] = dir[1];
        normal[1] = -dir[0];
    }
    
    // Apply offset to FIRST parameter (prev), not second (curr)
    // The direction is prev→curr, and we offset the START point (prev)
    output[0] = prev.x + normal[0] * comp_radius;
    output[1] = prev.y + normal[1] * comp_radius;
    
    // Debug output
    print_offset_calc(prev, curr, dir, normal, output);
    
    return true;
}

Gcode* CompensationPreprocessor::get_compensated_gcode()
{
    return serve_compensated_gcode(true);
}

void CompensationPreprocessor::flush()
{
    THEKERNEL->streams->printf(">>FLUSH: ENTRY - uncomp_count=%d comp_count=%d comp_ready=%d\n",
        uncomp_count, comp_count, comp_ready_count);

    is_flushing = true;

    if (!pipeline_primed && uncomp_count >= 2 && comp_ready_count == 0) {
        compute_and_output();
    }

    compute_terminal_output();

    uncomp_count = comp_ready_count;
    comp_count = comp_ready_count;

    THEKERNEL->streams->printf(">>FLUSH: EXIT - drain ready uncomp=%d comp=%d\n", uncomp_count, comp_count);
}

void CompensationPreprocessor::clear()
{
    THEKERNEL->streams->printf(">>CLEAR: Starting clear() - comp_count=%d, uncomp_count=%d\n", comp_count, uncomp_count);
    
    // Delete any remaining Gcode objects to prevent memory leaks
    for (int i = 0; i < BUFFER_SIZE; i++) {
        if (comp_ring[i].gcode != nullptr) {
            THEKERNEL->streams->printf(">>CLEAR: Deleting Gcode* at index %d (ptr=%p)\n", i, (void*)comp_ring[i].gcode);
            delete comp_ring[i].gcode;
            comp_ring[i].gcode = nullptr;
            THEKERNEL->streams->printf(">>CLEAR: Deleted and nullified index %d\n", i);
        }
    }
    
    THEKERNEL->streams->printf(">>CLEAR: All Gcode* objects deleted\n");
    
    // Reset buffer indices and counts
    uncomp_head = 0;
    uncomp_tail = 0;
    uncomp_count = 0;
    comp_head = 0;
    comp_tail = 0;
    comp_count = 0;
    comp_ready_count = 0;
    is_flushing = false;
    pipeline_primed = false;
    first_output_pending = true;
    has_initial_position = false;
    initial_position[0] = 0.0f;
    initial_position[1] = 0.0f;
    initial_position[2] = 0.0f;
    
    THEKERNEL->streams->printf(">>CLEAR: Buffer indices reset\n");
    
    // Zero out buffer memory
    for (int i = 0; i < BUFFER_SIZE; i++) {
        uncomp_ring[i].x = 0.0f;
        uncomp_ring[i].y = 0.0f;
        uncomp_ring[i].z = 0.0f;
        comp_ring[i].x = 0.0f;
        comp_ring[i].y = 0.0f;
        comp_ring[i].z = 0.0f;
    }
    
    THEKERNEL->streams->printf(">>CLEAR: Complete - buffers zeroed\n");
}

void CompensationPreprocessor::set_initial_position(const float position[3])
{
    initial_position[0] = position[X_AXIS];
    initial_position[1] = position[Y_AXIS];
    initial_position[2] = position[Z_AXIS];
    has_initial_position = true;
}

Gcode* CompensationPreprocessor::serve_compensated_gcode(bool draining)
{
    THEKERNEL->streams->printf(">>SERVE_GCODE: ENTRY - draining=%d comp_count=%d comp_ready=%d comp_tail=%d\n",
        draining ? 1 : 0, comp_count, comp_ready_count, comp_tail);

    if (comp_ready_count == 0) {
        THEKERNEL->streams->printf(">>SERVE_GCODE: No compensated moves available\n");
        return nullptr;
    }

    Gcode* result = comp_ring[comp_tail].gcode;
    comp_ring[comp_tail].gcode = nullptr;
    comp_tail = (comp_tail + 1) % BUFFER_SIZE;
    comp_ready_count--;

    if (draining) {
        if (comp_count > 0) {
            comp_count--;
        }
        if (uncomp_count > 0) {
            uncomp_count--;
        }
        if (comp_ready_count == 0 && uncomp_count == 0) {
            is_flushing = false;
        }
    }

    THEKERNEL->streams->printf(">>SERVE_GCODE: EXIT - returning %p, counts now uncomp=%d comp=%d ready=%d\n",
        (void*)result, uncomp_count, comp_count, comp_ready_count);

    return result;
}

// Debug print functions - Stub implementations for new dual buffer architecture
void CompensationPreprocessor::print_uncomp_buffered()
{
    // TODO: Implement debug output for uncomp buffer
}

void CompensationPreprocessor::print_offset_calc(const UncompPoint& prev, const UncompPoint& curr,
                                                   const float dir[2], const float normal[2], const float output[2])
{
    // TODO: Implement debug output for offset calculation
}

void CompensationPreprocessor::print_output(const char* gcode_str)
{
    // TODO: Implement debug output for gcode output
}

void CompensationPreprocessor::print_buffer_state()
{
    const char* comp_str = "NONE";
    if (comp_type == CompensationType::LEFT) comp_str = "LEFT";
    else if (comp_type == CompensationType::RIGHT) comp_str = "RIGHT";
    
    THEKERNEL->streams->printf(">>BUFFER_STATE: uncomp_count=%d, uncomp_head=%d, uncomp_tail=%d, comp_count=%d, comp=%s, radius=%.3f\n",
        uncomp_count, uncomp_head, uncomp_tail, comp_count, comp_str, comp_radius);
}

