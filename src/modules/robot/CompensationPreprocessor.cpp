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

void CompensationPreprocessor::reset_load_balance_metrics()
{
    load_balance_metrics.input_gcode_count = 0;
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
        idx_a = idx_b; // Onset degenerates to perpendicular offset at the first point.
        idx_c = (uncomp_tail + 1) % BUFFER_SIZE;
        COMPENSATION_TRACE_PRINTF(THEKERNEL->streams, ">>COMPUTE: FIRST POINT - A[%d]=B[%d], C[%d]\n", idx_a, idx_b, idx_c);
    } else {
        idx_b = uncomp_tail;
        idx_a = (uncomp_tail - 1 + BUFFER_SIZE) % BUFFER_SIZE;
        idx_c = (uncomp_tail + 1) % BUFFER_SIZE;
        COMPENSATION_TRACE_PRINTF(THEKERNEL->streams, ">>COMPUTE: CORNER POINT - A[%d], B[%d], C[%d]\n", idx_a, idx_b, idx_c);
    }

    const UncompPoint& a = uncomp_ring[idx_a];
    const UncompPoint& b = uncomp_ring[idx_b];
    const UncompPoint& c = uncomp_ring[idx_c];

    success = calculate_corner_intersection(a, b, c, offset);

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

    char gcode_str[128];
    snprintf(gcode_str, sizeof(gcode_str), "G1 X%.3f Y%.3f Z%.3f",
             comp_ring[comp_idx].x, comp_ring[comp_idx].y, comp_ring[comp_idx].z);

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

    float offset[2];
    bool success = calculate_corner_intersection(prev, curr, curr, offset);
    if (success) {
        comp_ring[comp_idx].x = offset[0];
        comp_ring[comp_idx].y = offset[1];
    } else {
        comp_ring[comp_idx].x = curr.x;
        comp_ring[comp_idx].y = curr.y;
    }
    comp_ring[comp_idx].z = curr.z;

    char gcode_str[128];
    snprintf(gcode_str, sizeof(gcode_str), "G1 X%.3f Y%.3f Z%.3f",
             comp_ring[comp_idx].x, comp_ring[comp_idx].y, comp_ring[comp_idx].z);

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

    float u_in[2] = { b.x - a.x, b.y - a.y };
    float u_out[2] = { c.x - b.x, c.y - b.y };

    float mag_in = sqrtf(u_in[0] * u_in[0] + u_in[1] * u_in[1]);
    float mag_out = sqrtf(u_out[0] * u_out[0] + u_out[1] * u_out[1]);

    if (mag_in < epsilon && mag_out < epsilon) {
        return false;
    }

    if (mag_in >= epsilon) {
        u_in[0] /= mag_in;
        u_in[1] /= mag_in;
    }

    if (mag_out >= epsilon) {
        u_out[0] /= mag_out;
        u_out[1] /= mag_out;
    }

    // Degenerate endpoint handling: if one direction is missing, reuse the other.
    if (mag_in < epsilon) {
        u_in[0] = u_out[0];
        u_in[1] = u_out[1];
    }
    if (mag_out < epsilon) {
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
        "COMP_LB: in=%lu gen=%lu srv=%lu compute=%lu prime_wait=%lu lb_wait=%lu empty_srv=%lu "
        "pull_req=%lu hit=%.1f%% miss=%.1f%% prod_vs_pull=%.1f%% "
        "samples=%lu avg_margin=%.2f min_margin=%ld max_margin=%ld "
        "max_uncomp=%u max_comp_ready=%u\n",
        (unsigned long)m.input_gcode_count,
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

