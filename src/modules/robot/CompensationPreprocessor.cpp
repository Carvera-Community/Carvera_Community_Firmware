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
#include "GcodeDispatch.h"
#include "libs/StreamOutput.h"
#include "StreamOutputPool.h"

#include <cstring>
#include <cmath>
#include <cstdio>

using namespace std;

void CompensationPreprocessor::reset_load_balance_metrics()
{
    load_balance_metrics.input_gcode_count = 0;
    load_balance_metrics.arc_input_count = 0;
    load_balance_metrics.generated_gcode_count = 0;
    load_balance_metrics.served_gcode_count = 0;
    load_balance_metrics.compute_count = 0;
    load_balance_metrics.priming_wait_count = 0;
    load_balance_metrics.load_balance_wait_count = 0;
    load_balance_metrics.empty_serve_count = 0;
    load_balance_metrics.sample_count = 0;
    load_balance_metrics.cumulative_ready_margin = 0;
    load_balance_metrics.min_ready_margin = 0;
    load_balance_metrics.max_ready_margin = 0;
    load_balance_metrics.max_uncomp_depth = 0;
    load_balance_metrics.max_comp_ready_depth = 0;
}

void CompensationPreprocessor::record_load_balance_sample()
{
    int ready_margin = comp_ready_count - uncomp_count;

    if (uncomp_count > load_balance_metrics.max_uncomp_depth) {
        load_balance_metrics.max_uncomp_depth = uncomp_count;
    }

    if (comp_ready_count > load_balance_metrics.max_comp_ready_depth) {
        load_balance_metrics.max_comp_ready_depth = comp_ready_count;
    }

    if (load_balance_metrics.sample_count == 0) {
        load_balance_metrics.min_ready_margin = ready_margin;
        load_balance_metrics.max_ready_margin = ready_margin;
    } else {
        if (ready_margin < load_balance_metrics.min_ready_margin) {
            load_balance_metrics.min_ready_margin = ready_margin;
        }

        if (ready_margin > load_balance_metrics.max_ready_margin) {
            load_balance_metrics.max_ready_margin = ready_margin;
        }
    }

    load_balance_metrics.sample_count++;
    load_balance_metrics.cumulative_ready_margin += ready_margin;
}

uint8_t CompensationPreprocessor::resolve_motion_g(const Gcode* gcode) const
{
    if (gcode != nullptr && gcode->has_g && gcode->g <= 3) {
        return static_cast<uint8_t>(gcode->g);
    }

    uint8_t modal_g = THEKERNEL->gcode_dispatch->get_modal_command();
    if (modal_g <= 3) {
        return modal_g;
    }

    return last_g;
}

bool CompensationPreprocessor::get_motion_direction(const float start[3], const UncompPoint& end, bool use_segment_end, float dir[2]) const
{
    const float epsilon = 0.0001f;

    if (is_arc_motion(end.motion_g)) {
        float center_x = start[X_AXIS] + end.i;
        float center_y = start[Y_AXIS] + end.j;
        float radius_x = use_segment_end ? (end.x - center_x) : (start[X_AXIS] - center_x);
        float radius_y = use_segment_end ? (end.y - center_y) : (start[Y_AXIS] - center_y);
        float mag = sqrtf(radius_x * radius_x + radius_y * radius_y);

        if (mag < epsilon) {
            return false;
        }

        radius_x /= mag;
        radius_y /= mag;

        if (end.motion_g == 2) {
            dir[0] = radius_y;
            dir[1] = -radius_x;
        } else {
            dir[0] = -radius_y;
            dir[1] = radius_x;
        }

        return true;
    }

    float dx = end.x - start[X_AXIS];
    float dy = end.y - start[Y_AXIS];
    float mag = sqrtf(dx * dx + dy * dy);
    if (mag < epsilon) {
        return false;
    }

    dir[0] = dx / mag;
    dir[1] = dy / mag;
    return true;
}

void CompensationPreprocessor::format_compensated_gcode(const float uncomp_start[3], const float comp_start[3], const UncompPoint& curr,
    const float comp_end[3], char* gcode_str, size_t gcode_str_size) const
{
    char feedrate_suffix[32] = {0};
    if (curr.has_feedrate) {
        snprintf(feedrate_suffix, sizeof(feedrate_suffix), " F%.3f", curr.feedrate);
    }

    if (is_arc_motion(curr.motion_g)) {
        // Arc center in world space (uncompensated)
        float center_x = uncomp_start[X_AXIS] + curr.i;
        float center_y = uncomp_start[Y_AXIS] + curr.j;

        // I/J offsets from the compensated arc start to the world center
        float new_i = center_x - comp_start[X_AXIS];
        float new_j = center_y - comp_start[Y_AXIS];

        // Derive eff_radius from the uncompensated arc geometry rather than from
        // comp_start so that positional errors in comp_start (e.g. a miter junction
        // that does not land exactly on the ideal offset circle) are not inherited.
        // Rule: G2+G41 or G3+G42 -> outward (larger radius);
        //       G3+G41 or G2+G42 -> inward (smaller radius).
        const float epsilon = 0.0001f;
        float uncomp_radius = sqrtf(
            (center_x - uncomp_start[X_AXIS]) * (center_x - uncomp_start[X_AXIS]) +
            (center_y - uncomp_start[Y_AXIS]) * (center_y - uncomp_start[Y_AXIS]));
        bool outward = (curr.motion_g == 2 && comp_type == CompensationType::LEFT) ||
                       (curr.motion_g == 3 && comp_type == CompensationType::RIGHT);
        float eff_radius = outward ? (uncomp_radius + comp_radius)
                                   : (uncomp_radius - comp_radius);
        if (eff_radius < 0.0f) eff_radius = 0.0f;
        // Direction from world center toward the uncompensated endpoint
        float ex = curr.x - center_x;
        float ey = curr.y - center_y;
        float em = sqrtf(ex * ex + ey * ey);

        float arc_end_x = comp_end[X_AXIS];
        float arc_end_y = comp_end[Y_AXIS];
        if (em > epsilon && eff_radius > epsilon) {
            // Place the compensated endpoint exactly on the offset circle,
            // in the radial direction of the uncompensated endpoint.
            arc_end_x = center_x + (ex / em) * eff_radius;
            arc_end_y = center_y + (ey / em) * eff_radius;
        }

        snprintf(gcode_str, gcode_str_size, "G%d X%.3f Y%.3f Z%.3f I%.3f J%.3f%s",
            curr.motion_g,
            arc_end_x,
            arc_end_y,
            comp_end[Z_AXIS],
            new_i,
            new_j,
            feedrate_suffix);
        return;
    }

    snprintf(gcode_str, gcode_str_size, "G%d X%.3f Y%.3f Z%.3f%s",
        curr.motion_g <= 1 ? curr.motion_g : 1,
        comp_end[X_AXIS],
        comp_end[Y_AXIS],
        comp_end[Z_AXIS],
        feedrate_suffix);
}

CompensationPreprocessor::CompensationPreprocessor()
{
    reset_load_balance_metrics();

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
        uncomp_ring[i].i = 0.0f;
        uncomp_ring[i].j = 0.0f;
        uncomp_ring[i].feedrate = 0.0f;
        uncomp_ring[i].motion_g = 0;
        uncomp_ring[i].has_feedrate = false;
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
    COMPENSATION_TRACE_PRINTF(THEKERNEL->streams, ">>SET_COMP: ENTRY - type=%d, radius=%.3f\n", (int)type, radius);
    
    comp_type = type;
    comp_radius = radius;
    
    if (type == CompensationType::NONE) {
        // G40 - Turn off compensation after explicit flush has drained buffers
        is_flushing = false;
        comp_active = false;
        pipeline_primed = false;
        first_output_pending = true;
        has_initial_position = false;
        COMPENSATION_TRACE_PRINTF(THEKERNEL->streams, ">>PHASE2: G40 activated - compensation OFF\n");
    } else {
        // G41/G42 - Turn on compensation
        COMPENSATION_TRACE_PRINTF(THEKERNEL->streams, ">>PHASE2: G41/G42 called - clearing buffer (count was %d)\n", uncomp_count);
        clear();
        COMPENSATION_TRACE_PRINTF(THEKERNEL->streams, ">>SET_COMP: clear() returned successfully\n");
        
        is_flushing = false;
        comp_active = true;
        pipeline_primed = false;
        first_output_pending = true;
        has_initial_position = false;
        
        const char* side_str = (type == CompensationType::LEFT) ? "LEFT (G41)" : "RIGHT (G42)";
        COMPENSATION_TRACE_PRINTF(THEKERNEL->streams, ">>PHASE2: Compensation activated - type=%s, radius=%.3f\n", side_str, radius);
    }
    
    COMPENSATION_TRACE_PRINTF(THEKERNEL->streams, ">>SET_COMP: EXIT - comp_active=%d\n", comp_active);
}

Gcode* CompensationPreprocessor::buffer_gcode(Gcode* gcode)
{
    load_balance_metrics.input_gcode_count++;
    COMPENSATION_TRACE_PRINTF(THEKERNEL->streams, ">>BUFFER_GCODE: ENTRY - gcode=%p, uncomp_count=%d, comp_ready=%d\n",
        (void*)gcode, uncomp_count, comp_ready_count);

    if (gcode == nullptr) {
        load_balance_metrics.load_balance_wait_count++;
        record_load_balance_sample();
        COMPENSATION_TRACE_PRINTF(THEKERNEL->streams, ">>BUFFER_GCODE: Null gcode pointer\n");
        return nullptr;
    }

    uint8_t motion_g = resolve_motion_g(gcode);
    last_g = motion_g;
    if (is_arc_motion(motion_g)) {
        load_balance_metrics.arc_input_count++;
    }
    
    // Extract coordinates (modal if not specified)
    float x, y, z;
    
    if (uncomp_count == 0) {
        // First point - use explicit coordinates or captured activation position
        COMPENSATION_TRACE_PRINTF(THEKERNEL->streams, ">>BUFFER_GCODE: First move after compensation activation\n");
        x = gcode->has_letter('X') ? gcode->get_value('X') : initial_position[X_AXIS];
        y = gcode->has_letter('Y') ? gcode->get_value('Y') : initial_position[Y_AXIS];
        z = gcode->has_letter('Z') ? gcode->get_value('Z') : initial_position[Z_AXIS];
        COMPENSATION_TRACE_PRINTF(THEKERNEL->streams, ">>BUFFER_GCODE: Coords X=%.3f Y=%.3f Z=%.3f\n", x, y, z);
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
    uncomp_ring[uncomp_head].i = gcode->has_letter('I') ? gcode->get_value('I') : 0.0f;
    uncomp_ring[uncomp_head].j = gcode->has_letter('J') ? gcode->get_value('J') : 0.0f;
    uncomp_ring[uncomp_head].feedrate = gcode->has_letter('F') ? gcode->get_value('F') : 0.0f;
    uncomp_ring[uncomp_head].motion_g = motion_g;
    uncomp_ring[uncomp_head].has_feedrate = gcode->has_letter('F');
    
    uncomp_head = (uncomp_head + 1) % BUFFER_SIZE;
    if (uncomp_count < BUFFER_SIZE) {
        uncomp_count++;
    }
    
    print_uncomp_buffered();

    if (uncomp_count < BUFFER_SIZE) {
        load_balance_metrics.priming_wait_count++;
        load_balance_metrics.load_balance_wait_count++;
        record_load_balance_sample();
        COMPENSATION_TRACE_PRINTF(THEKERNEL->streams, ">>BUFFER_GCODE: Priming uncomp buffer (count=%d)\n", uncomp_count);
        return nullptr;
    }

    if (!pipeline_primed) {
        load_balance_metrics.priming_wait_count++;
        load_balance_metrics.load_balance_wait_count++;
        COMPENSATION_TRACE_PRINTF(THEKERNEL->streams, ">>BUFFER_GCODE: First full cycle - priming with double compute\n");
        compute_and_output();
        if (comp_ready_count < BUFFER_SIZE) {
            compute_and_output();
        }
        pipeline_primed = true;
        print_buffer_state();
        record_load_balance_sample();
        return nullptr;
    }
    
    if (comp_ready_count < BUFFER_SIZE) {
        COMPENSATION_TRACE_PRINTF(THEKERNEL->streams, ">>BUFFER_GCODE: Steady-state single compute\n");
        compute_and_output();
    }

    Gcode* output = serve_compensated_gcode(false);
    if (output == nullptr) {
        load_balance_metrics.load_balance_wait_count++;
    }
    record_load_balance_sample();
    COMPENSATION_TRACE_PRINTF(THEKERNEL->streams, ">>BUFFER_GCODE: EXIT - returning %p\n", (void*)output);
    return output;
}

// ============================================================================
// PHASE-LOCKED DUAL BUFFER PROCESSING
// ============================================================================

void CompensationPreprocessor::compute_and_output()
{
    load_balance_metrics.compute_count++;
    COMPENSATION_TRACE_PRINTF(THEKERNEL->streams, ">>COMPUTE: ENTRY - uncomp_count=%d, comp_count=%d, comp_ready=%d\n",
        uncomp_count, comp_count, comp_ready_count);

    if (uncomp_count < 2) {
        record_load_balance_sample();
        COMPENSATION_TRACE_PRINTF(THEKERNEL->streams, ">>COMPUTE: Not enough uncomp points to compute\n");
        return;
    }

    if (comp_ready_count >= BUFFER_SIZE) {
        record_load_balance_sample();
        COMPENSATION_TRACE_PRINTF(THEKERNEL->streams, ">>COMPUTE: Comp ring has no free slot, skip\n");
        return;
    }

    int comp_idx = comp_head;
    if (comp_ring[comp_idx].gcode != nullptr) {
        record_load_balance_sample();
        COMPENSATION_TRACE_PRINTF(THEKERNEL->streams, ">>COMPUTE: Target comp slot %d still occupied, skip\n", comp_idx);
        return;
    }

    float offset[2];
    bool success = false;
    int idx_a = 0;
    int idx_b = 0;
    int idx_c = 0;

    if (first_output_pending) {
        idx_b = uncomp_tail;
        idx_a = idx_b; // kept only so uncomp_start ternary below compiles; actual intersection uses pseudo_prev
        idx_c = (uncomp_tail + 1) % BUFFER_SIZE;
        COMPENSATION_TRACE_PRINTF(THEKERNEL->streams, ">>COMPUTE: FIRST POINT - B[%d], C[%d]\n", idx_b, idx_c);
    } else {
        idx_b = uncomp_tail;
        idx_a = (uncomp_tail - 1 + BUFFER_SIZE) % BUFFER_SIZE;
        idx_c = (uncomp_tail + 1) % BUFFER_SIZE;
        COMPENSATION_TRACE_PRINTF(THEKERNEL->streams, ">>COMPUTE: CORNER POINT - A[%d], B[%d], C[%d]\n", idx_a, idx_b, idx_c);
    }

    const UncompPoint& a = uncomp_ring[idx_a];
    const UncompPoint& b = uncomp_ring[idx_b];
    const UncompPoint& c = uncomp_ring[idx_c];

    if (first_output_pending) {
        // Use initial_position as the incoming start so arc tangents are computed from the
        // correct arc start.  With idx_a==idx_b the old path used arc_endpoint+I/J as the
        // center, which is wrong for arcs and produced a bad u_in → wrong miter corner.
        UncompPoint pseudo_prev;
        pseudo_prev.x = initial_position[X_AXIS];
        pseudo_prev.y = initial_position[Y_AXIS];
        pseudo_prev.z = initial_position[Z_AXIS];
        pseudo_prev.i = 0.0f;
        pseudo_prev.j = 0.0f;
        pseudo_prev.motion_g = 1;
        pseudo_prev.has_feedrate = false;
        pseudo_prev.feedrate = 0.0f;
        success = calculate_corner_intersection(pseudo_prev, b, c, offset);
    } else {
        success = calculate_corner_intersection(a, b, c, offset);
    }

    if (success) {
        comp_ring[comp_idx].x = offset[0];
        comp_ring[comp_idx].y = offset[1];
        comp_ring[comp_idx].z = b.z;
        COMPENSATION_TRACE_PRINTF(THEKERNEL->streams, ">>COMPUTE: Offset result - comp[%d]=(%.3f,%.3f,%.3f)\n",
            comp_idx, offset[0], offset[1], b.z);
    } else {
        comp_ring[comp_idx].x = b.x;
        comp_ring[comp_idx].y = b.y;
        comp_ring[comp_idx].z = b.z;
        COMPENSATION_TRACE_PRINTF(THEKERNEL->streams, ">>COMPUTE: Degenerate motion - no offset\n");
    }

    float uncomp_start[3] = { first_output_pending ? initial_position[X_AXIS] : a.x,
                               first_output_pending ? initial_position[Y_AXIS] : a.y,
                               first_output_pending ? initial_position[Z_AXIS] : a.z };
    float comp_start[3] = { initial_position[X_AXIS], initial_position[Y_AXIS], initial_position[Z_AXIS] };
    if (!first_output_pending) {
        int prev_comp_idx = (comp_head - 1 + BUFFER_SIZE) % BUFFER_SIZE;
        comp_start[X_AXIS] = comp_ring[prev_comp_idx].x;
        comp_start[Y_AXIS] = comp_ring[prev_comp_idx].y;
        comp_start[Z_AXIS] = comp_ring[prev_comp_idx].z;
    }
    float comp_end[3] = { comp_ring[comp_idx].x, comp_ring[comp_idx].y, comp_ring[comp_idx].z };

    char gcode_str[128];
    format_compensated_gcode(uncomp_start, comp_start, b, comp_end, gcode_str, sizeof(gcode_str));

    print_output(gcode_str);

    comp_ring[comp_idx].gcode = new Gcode(gcode_str, &StreamOutput::NullStream);
    load_balance_metrics.generated_gcode_count++;

    if (comp_count < BUFFER_SIZE) {
        comp_count++;
    }
    comp_ready_count++;
    uncomp_tail = (uncomp_tail + 1) % BUFFER_SIZE;
    comp_head = (comp_head + 1) % BUFFER_SIZE;
    first_output_pending = false;
    record_load_balance_sample();

    COMPENSATION_TRACE_PRINTF(THEKERNEL->streams, ">>COMPUTE: EXIT - complete, comp_count=%d, comp_ready=%d, uncomp_tail=%d, comp_head=%d\n",
        comp_count, comp_ready_count, uncomp_tail, comp_head);
}

bool CompensationPreprocessor::compute_terminal_output()
{
    if (uncomp_count == 0 || comp_ready_count >= BUFFER_SIZE) {
        return false;
    }

    // Special case: single move between G41 and G40 with no look-ahead (e.g. bare G41+arc+G40).
    // compute_and_output never fires (needs uncomp_count>=2), so handle it here.
    if (uncomp_count == 1 && first_output_pending) {
        int last_idx = (uncomp_head - 1 + BUFFER_SIZE) % BUFFER_SIZE;
        int comp_idx = comp_head;

        if (comp_ring[comp_idx].gcode != nullptr) {
            record_load_balance_sample();
            return false;
        }

        const UncompPoint& curr = uncomp_ring[last_idx];

        // Tangent at the end of the move, starting from initial_position.
        float dir[2] = { 0.0f, 0.0f };
        bool has_dir = get_motion_direction(initial_position, curr, true, dir);

        float n_x = 0.0f, n_y = 0.0f;
        if (has_dir) {
            if (comp_type == CompensationType::LEFT) {
                n_x = -dir[1]; n_y = dir[0];
            } else {
                n_x = dir[1]; n_y = -dir[0];
            }
        }

        comp_ring[comp_idx].x = curr.x + n_x * comp_radius;
        comp_ring[comp_idx].y = curr.y + n_y * comp_radius;
        comp_ring[comp_idx].z = curr.z;

        float uncomp_start[3] = { initial_position[X_AXIS], initial_position[Y_AXIS], initial_position[Z_AXIS] };
        float comp_start[3]   = { initial_position[X_AXIS], initial_position[Y_AXIS], initial_position[Z_AXIS] };
        float comp_end[3]     = { comp_ring[comp_idx].x, comp_ring[comp_idx].y, comp_ring[comp_idx].z };

        char gcode_str[128];
        format_compensated_gcode(uncomp_start, comp_start, curr, comp_end, gcode_str, sizeof(gcode_str));

        comp_ring[comp_idx].gcode = new Gcode(gcode_str, &StreamOutput::NullStream);
        load_balance_metrics.generated_gcode_count++;

        comp_head = (comp_head + 1) % BUFFER_SIZE;
        comp_ready_count++;
        if (comp_count < BUFFER_SIZE) comp_count++;
        uncomp_tail = uncomp_head;
        record_load_balance_sample();

        COMPENSATION_TRACE_PRINTF(THEKERNEL->streams, ">>TERMINAL: Single-move case comp[%d] generated\n", comp_idx);
        return true;
    }

    if (uncomp_count < 2 || comp_ready_count >= BUFFER_SIZE) {
        return false;
    }

    int prev_idx = (uncomp_head - 2 + BUFFER_SIZE) % BUFFER_SIZE;
    int last_idx = (uncomp_head - 1 + BUFFER_SIZE) % BUFFER_SIZE;
    int comp_idx = comp_head;

    if (comp_ring[comp_idx].gcode != nullptr) {
        record_load_balance_sample();
        COMPENSATION_TRACE_PRINTF(THEKERNEL->streams, ">>TERMINAL: Target comp slot %d still occupied, skip\n", comp_idx);
        return false;
    }

    const UncompPoint& prev = uncomp_ring[prev_idx];
    const UncompPoint& curr = uncomp_ring[last_idx];

    // Terminal point has no outgoing segment; offset from the endpoint using
    // the incoming move tangent (line or arc) evaluated at the segment end.
    float start_in[3] = { prev.x, prev.y, prev.z };
    float dir[2] = { 0.0f, 0.0f };
    bool has_dir = get_motion_direction(start_in, curr, true, dir);
    if (has_dir) {
        float n_x = 0.0f;
        float n_y = 0.0f;
        if (comp_type == CompensationType::LEFT) {
            n_x = -dir[1];
            n_y = dir[0];
        } else {
            n_x = dir[1];
            n_y = -dir[0];
        }
        comp_ring[comp_idx].x = curr.x + n_x * comp_radius;
        comp_ring[comp_idx].y = curr.y + n_y * comp_radius;
    } else {
        comp_ring[comp_idx].x = curr.x;
        comp_ring[comp_idx].y = curr.y;
    }
    comp_ring[comp_idx].z = curr.z;

    float uncomp_start[3] = { prev.x, prev.y, prev.z };
    float comp_start[3] = { initial_position[X_AXIS], initial_position[Y_AXIS], initial_position[Z_AXIS] };
    if (comp_ready_count > 0 || comp_count > 0) {
        int prev_comp_idx = (comp_head - 1 + BUFFER_SIZE) % BUFFER_SIZE;
        comp_start[X_AXIS] = comp_ring[prev_comp_idx].x;
        comp_start[Y_AXIS] = comp_ring[prev_comp_idx].y;
        comp_start[Z_AXIS] = comp_ring[prev_comp_idx].z;
    }
    float comp_end[3] = { comp_ring[comp_idx].x, comp_ring[comp_idx].y, comp_ring[comp_idx].z };

    char gcode_str[128];
    format_compensated_gcode(uncomp_start, comp_start, curr, comp_end, gcode_str, sizeof(gcode_str));

    comp_ring[comp_idx].gcode = new Gcode(gcode_str, &StreamOutput::NullStream);
    load_balance_metrics.generated_gcode_count++;

    comp_head = (comp_head + 1) % BUFFER_SIZE;
    comp_ready_count++;
    if (comp_count < BUFFER_SIZE) {
        comp_count++;
    }
    uncomp_tail = uncomp_head;
    record_load_balance_sample();

    COMPENSATION_TRACE_PRINTF(THEKERNEL->streams, ">>TERMINAL: Added final point comp[%d], comp_ready=%d\n", comp_idx, comp_ready_count);
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

bool CompensationPreprocessor::calculate_corner_intersection(
    const UncompPoint& a,
    const UncompPoint& b,
    const UncompPoint& c,
    float output[2])
{
    const float epsilon = 0.0001f;

    float start_in[3] = { a.x, a.y, a.z };
    float start_out[3] = { b.x, b.y, b.z };
    float u_in[2] = { 0.0f, 0.0f };
    float u_out[2] = { 0.0f, 0.0f };

    bool has_in = get_motion_direction(start_in, b, true, u_in);
    bool has_out = get_motion_direction(start_out, c, false, u_out);

    if (!has_in && !has_out) {
        return false;
    }

    // Degenerate endpoint handling: if one direction is missing, reuse the other.
    if (!has_in) {
        u_in[0] = u_out[0];
        u_in[1] = u_out[1];
    }
    if (!has_out) {
        u_out[0] = u_in[0];
        u_out[1] = u_in[1];
    }

    float n_in[2];
    float n_out[2];
    if (comp_type == CompensationType::LEFT) {
        n_in[0] = -u_in[1];
        n_in[1] = u_in[0];
        n_out[0] = -u_out[1];
        n_out[1] = u_out[0];
    } else {
        n_in[0] = u_in[1];
        n_in[1] = -u_in[0];
        n_out[0] = u_out[1];
        n_out[1] = -u_out[0];
    }

    float p1[2] = { b.x + n_in[0] * comp_radius, b.y + n_in[1] * comp_radius };
    float p2[2] = { b.x + n_out[0] * comp_radius, b.y + n_out[1] * comp_radius };

    float denom = u_in[0] * u_out[1] - u_in[1] * u_out[0];
    if (fabsf(denom) < epsilon) {
        output[0] = p1[0];
        output[1] = p1[1];
        return true;
    }

    float p2_minus_p1[2] = { p2[0] - p1[0], p2[1] - p1[1] };
    float s = cross_product_2d(p2_minus_p1, u_out) / denom;

    output[0] = p1[0] + s * u_in[0];
    output[1] = p1[1] + s * u_in[1];

    return true;
}

Gcode* CompensationPreprocessor::get_compensated_gcode()
{
    return serve_compensated_gcode(true);
}

void CompensationPreprocessor::print_load_balance_report(StreamOutput* stream) const
{
    const LoadBalanceMetrics& m = load_balance_metrics;
    uint32_t pull_requests = m.served_gcode_count + m.empty_serve_count;
    float avg_margin = (m.sample_count > 0)
        ? (float)m.cumulative_ready_margin / (float)m.sample_count
        : 0.0f;
    float pull_hit_rate = (pull_requests > 0)
        ? ((float)m.served_gcode_count * 100.0f) / (float)pull_requests
        : 100.0f;
    float pull_miss_rate = (pull_requests > 0)
        ? ((float)m.empty_serve_count * 100.0f) / (float)pull_requests
        : 0.0f;
    float prod_vs_pull = (pull_requests > 0)
        ? ((float)m.generated_gcode_count * 100.0f) / (float)pull_requests
        : 100.0f;

    stream->printf(
        "COMP_LB: in=%lu arc_in=%lu gen=%lu srv=%lu compute=%lu prime_wait=%lu lb_wait=%lu empty_srv=%lu "
        "pull_req=%lu hit=%.1f%% miss=%.1f%% prod_vs_pull=%.1f%% "
        "samples=%lu avg_margin=%.2f min_margin=%ld max_margin=%ld "
        "max_uncomp=%u max_comp_ready=%u\n",
        (unsigned long)m.input_gcode_count,
        (unsigned long)m.arc_input_count,
        (unsigned long)m.generated_gcode_count,
        (unsigned long)m.served_gcode_count,
        (unsigned long)m.compute_count,
        (unsigned long)m.priming_wait_count,
        (unsigned long)m.load_balance_wait_count,
        (unsigned long)m.empty_serve_count,
        (unsigned long)pull_requests,
        pull_hit_rate,
        pull_miss_rate,
        prod_vs_pull,
        (unsigned long)m.sample_count,
        avg_margin,
        (long)m.min_ready_margin,
        (long)m.max_ready_margin,
        (unsigned)m.max_uncomp_depth,
        (unsigned)m.max_comp_ready_depth
    );
}

void CompensationPreprocessor::flush()
{
    COMPENSATION_TRACE_PRINTF(THEKERNEL->streams, ">>FLUSH: ENTRY - uncomp_count=%d comp_count=%d comp_ready=%d\n",
        uncomp_count, comp_count, comp_ready_count);

    is_flushing = true;

    if (!pipeline_primed && uncomp_count >= 2 && comp_ready_count == 0) {
        compute_and_output();
    }

    compute_terminal_output();

    uncomp_count = comp_ready_count;
    comp_count = comp_ready_count;
    record_load_balance_sample();

    COMPENSATION_TRACE_PRINTF(THEKERNEL->streams, ">>FLUSH: EXIT - drain ready uncomp=%d comp=%d\n", uncomp_count, comp_count);
}

void CompensationPreprocessor::clear()
{
    COMPENSATION_TRACE_PRINTF(THEKERNEL->streams, ">>CLEAR: Starting clear() - comp_count=%d, uncomp_count=%d\n", comp_count, uncomp_count);
    
    // Delete any remaining Gcode objects to prevent memory leaks
    for (int i = 0; i < BUFFER_SIZE; i++) {
        if (comp_ring[i].gcode != nullptr) {
            COMPENSATION_TRACE_PRINTF(THEKERNEL->streams, ">>CLEAR: Deleting Gcode* at index %d (ptr=%p)\n", i, (void*)comp_ring[i].gcode);
            delete comp_ring[i].gcode;
            comp_ring[i].gcode = nullptr;
            COMPENSATION_TRACE_PRINTF(THEKERNEL->streams, ">>CLEAR: Deleted and nullified index %d\n", i);
        }
    }
    
    COMPENSATION_TRACE_PRINTF(THEKERNEL->streams, ">>CLEAR: All Gcode* objects deleted\n");
    
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
    record_load_balance_sample();
    
    COMPENSATION_TRACE_PRINTF(THEKERNEL->streams, ">>CLEAR: Buffer indices reset\n");
    
    // Zero out buffer memory
    for (int i = 0; i < BUFFER_SIZE; i++) {
        uncomp_ring[i].x = 0.0f;
        uncomp_ring[i].y = 0.0f;
        uncomp_ring[i].z = 0.0f;
        uncomp_ring[i].i = 0.0f;
        uncomp_ring[i].j = 0.0f;
        uncomp_ring[i].feedrate = 0.0f;
        uncomp_ring[i].motion_g = 0;
        uncomp_ring[i].has_feedrate = false;
        comp_ring[i].x = 0.0f;
        comp_ring[i].y = 0.0f;
        comp_ring[i].z = 0.0f;
    }
    
    COMPENSATION_TRACE_PRINTF(THEKERNEL->streams, ">>CLEAR: Complete - buffers zeroed\n");
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
    COMPENSATION_TRACE_PRINTF(THEKERNEL->streams, ">>SERVE_GCODE: ENTRY - draining=%d comp_count=%d comp_ready=%d comp_tail=%d\n",
        draining ? 1 : 0, comp_count, comp_ready_count, comp_tail);

    if (comp_ready_count == 0) {
        load_balance_metrics.empty_serve_count++;
        record_load_balance_sample();
        COMPENSATION_TRACE_PRINTF(THEKERNEL->streams, ">>SERVE_GCODE: No compensated moves available\n");
        return nullptr;
    }

    Gcode* result = comp_ring[comp_tail].gcode;
    comp_ring[comp_tail].gcode = nullptr;
    comp_tail = (comp_tail + 1) % BUFFER_SIZE;
    comp_ready_count--;
    load_balance_metrics.served_gcode_count++;

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
    record_load_balance_sample();

    COMPENSATION_TRACE_PRINTF(THEKERNEL->streams, ">>SERVE_GCODE: EXIT - returning %p, counts now uncomp=%d comp=%d ready=%d\n",
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

    COMPENSATION_TRACE_PRINTF(THEKERNEL->streams, ">>BUFFER_STATE: uncomp_count=%d, uncomp_head=%d, uncomp_tail=%d, comp_count=%d, comp=%s, radius=%.3f\n",
        uncomp_count, uncomp_head, uncomp_tail, comp_count, comp_str, comp_radius);
}

