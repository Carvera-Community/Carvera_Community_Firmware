#include "CompensationPreprocessor.h"
#include "Robot.h"
#include "libs/StreamOutputPool.h"
#include "libs/Kernel.h"
#include <cmath>
#include <queue>

CompensationPreprocessor::CompensationPreprocessor() 
    : comp_side(Compensation::NONE), 
      comp_radius(0),
      first_move_after_enable(false)
{
    last_position[0] = last_position[1] = last_position[2] = 0;
    last_uncompensated_position[0] = last_uncompensated_position[1] = last_uncompensated_position[2] = 0;
    THEKERNEL->streams->printf("DBG:CompPrep: Preprocessor initialized with lookahead buffering\n");
}

CompensationPreprocessor::~CompensationPreprocessor() {
    while (!move_buffer.empty()) {
        move_buffer.pop();
    }
}

// ============================================================================
// Compensation Control
// ============================================================================

void CompensationPreprocessor::enable_compensation(CompSide side, float diameter) {
    if (side != Compensation::NONE && side != Compensation::LEFT && side != Compensation::RIGHT) {
        THEKERNEL->streams->printf("E02:InvalidSide\n"); // Error: invalid compensation side
        return;
    }

    comp_side = side;
    comp_radius = diameter / 2.0f;
    first_move_after_enable = true;
    
    while (!move_buffer.empty()) {
        move_buffer.pop();
    }
    
    // Silent enable - no confirmation message
}

void CompensationPreprocessor::disable_compensation() {
    if (comp_side != Compensation::NONE) {
        // Silent disable
        comp_side = Compensation::NONE;
        comp_radius = 0;
        first_move_after_enable = false;
    }
}

// ============================================================================
// Buffer Management
// ============================================================================

bool CompensationPreprocessor::buffer_move(const ParsedMove& move) {
    if (!move.valid) {
        THEKERNEL->streams->printf("E01\n"); // Error code: invalid move
        return false;
    }
    
    // Calculate move geometry from UNCOMPENSATED position to check if it's zero-length
    ParsedMove temp_move = move;
    calculate_move_geometry(temp_move, last_uncompensated_position);
    
    // Skip zero-length moves (they would cause division by zero in intersection calc)
    if (temp_move.length_2d < 0.001f) {
        // Silent skip - update Z if needed
        last_uncompensated_position[2] = move.xyz[2];
        last_position[2] = move.xyz[2];
        THEKERNEL->streams->printf("Z\n"); // Compact: Zero-length skipped
        return true;
    }
    
    move_buffer.push(temp_move);  // Push the move with calculated geometry
    THEKERNEL->streams->printf("B%d\n", move_buffer.size()); // Compact: Buffered, buffer size
    
    // Update uncompensated position to this move's endpoint
    last_uncompensated_position[0] = move.xyz[0];
    last_uncompensated_position[1] = move.xyz[1];
    last_uncompensated_position[2] = move.xyz[2];
    
    return true;
}

bool CompensationPreprocessor::get_compensated_move(ParsedMove& output_move) {
    if (move_buffer.size() < LOOKAHEAD_DEPTH) {
        return false; // Silent - waiting for more moves
    }
    
    std::queue<ParsedMove> temp_queue = move_buffer;
    
    ParsedMove move1 = temp_queue.front(); temp_queue.pop();
    ParsedMove move2 = temp_queue.front(); temp_queue.pop();
    ParsedMove move3 = temp_queue.front();
    
    move_buffer.pop();
    
    THEKERNEL->streams->printf("C\n"); // Compact: Compensating move
    
    output_move = move1;
    
    // Geometry already calculated during buffering, but verify direction is from last_position
    // (in case buffer was flushed and last_position changed)
    calculate_move_geometry(output_move, last_position);
    
    if (move1.mode == MOTION_LINEAR || move1.mode == MOTION_SEEK) {
        if (move2.mode == MOTION_LINEAR || move2.mode == MOTION_SEEK) {
            // move2 geometry already calculated from move1.xyz during buffering
            // But recalculate to ensure it's relative to move1's compensated endpoint
            ParsedMove temp_move2 = move2;
            calculate_move_geometry(temp_move2, move1.xyz);
            
            float dir_dot = output_move.direction[0] * temp_move2.direction[0] + 
                          output_move.direction[1] * temp_move2.direction[1];
            
            if (dir_dot < 0.9999f) {
                // Corner detected
                THEKERNEL->streams->printf("K\n"); // Compact: Corner detected
                
                float intersection[2];
                if (calculate_corner_intersection(output_move, temp_move2, intersection)) {
                    output_move.xyz[0] = intersection[0];
                    output_move.xyz[1] = intersection[1];
                    THEKERNEL->streams->printf("I\n"); // Compact: Intersection applied
                } else {
                    THEKERNEL->streams->printf("W01\n"); // Warning: corner calc failed
                    float offset[2];
                    calculate_perpendicular_offset(output_move, last_position, offset);
                    output_move.xyz[0] = offset[0];
                    output_move.xyz[1] = offset[1];
                }
            } else {
                THEKERNEL->streams->printf("L\n"); // Compact: coLlinear moves
                float offset[2];
                calculate_perpendicular_offset(move1, last_position, offset);
                output_move.xyz[0] = offset[0];
                output_move.xyz[1] = offset[1];
            }
        } else {
            THEKERNEL->streams->printf("S\n"); // Compact: Simple offset
            float offset[2];
            calculate_perpendicular_offset(move1, last_position, offset);
            output_move.xyz[0] = offset[0];
            output_move.xyz[1] = offset[1];
        }
        
    } else if (move1.mode == MOTION_CW_ARC || move1.mode == MOTION_CCW_ARC) {
        THEKERNEL->streams->printf("A\n"); // Compact: Arc compensation
        compensate_arc(output_move);
    }
    
    last_position[0] = output_move.xyz[0];
    last_position[1] = output_move.xyz[1];
    last_position[2] = output_move.xyz[2];
    
    THEKERNEL->streams->printf("X\n"); // Compact: eXecuting move
    
    return true;
}

void CompensationPreprocessor::flush_buffer(std::queue<ParsedMove>& flushed_moves) {
    THEKERNEL->streams->printf("F%d\n", move_buffer.size()); // Compact: Flushing N moves
    
    while (!move_buffer.empty()) {
        ParsedMove move = move_buffer.front();
        move_buffer.pop();
        
        if (move.mode == MOTION_LINEAR || move.mode == MOTION_SEEK) {
            calculate_move_geometry(move, last_position);
            float offset[2];
            calculate_perpendicular_offset(move, last_position, offset);
            move.xyz[0] = offset[0];
            move.xyz[1] = offset[1];
            
            last_position[0] = move.xyz[0];
            last_position[1] = move.xyz[1];
            last_position[2] = move.xyz[2];
        } else if (move.mode == MOTION_CW_ARC || move.mode == MOTION_CCW_ARC) {
            compensate_arc(move);
        }
        
        flushed_moves.push(move);
        // Silent - no flushed move output
    }
}

// ============================================================================
// Core Compensation Algorithms
// ============================================================================

void CompensationPreprocessor::calculate_perpendicular_offset(
    const ParsedMove& move,
    const float start_pos[2],
    float output_xy[2]
) const {
    float ux = move.direction[0];
    float uy = move.direction[1];
    
    if (move.length_2d == 0 || (ux == 0 && uy == 0)) {
        float dx = move.xyz[0] - start_pos[0];
        float dy = move.xyz[1] - start_pos[1];
        float len = sqrtf(dx*dx + dy*dy);
        
        if (len < 0.00001f) {
            output_xy[0] = move.xyz[0];
            output_xy[1] = move.xyz[1];
            // Silent - zero-length move, no offset
            return;
        }
        
        ux = dx / len;
        uy = dy / len;
    }
    
    float nx, ny;
    if (comp_side == Compensation::LEFT) {
        nx = -uy;
        ny = ux;
    } else {
        nx = uy;
        ny = -ux;
    }
    
    output_xy[0] = move.xyz[0] + nx * comp_radius;
    output_xy[1] = move.xyz[1] + ny * comp_radius;
    
    // Silent - offset calculated
}

bool CompensationPreprocessor::calculate_corner_intersection(
    const ParsedMove& move1,
    const ParsedMove& move2,
    float intersection_xy[2]
) const {
    if ((move1.mode != MOTION_LINEAR && move1.mode != MOTION_SEEK) ||
        (move2.mode != MOTION_LINEAR && move2.mode != MOTION_SEEK)) {
        return false;
    }
    
    float u1x = move1.direction[0];
    float u1y = move1.direction[1];
    float u2x = move2.direction[0];
    float u2y = move2.direction[1];
    
    float n1x, n1y, n2x, n2y;
    if (comp_side == Compensation::LEFT) {
        n1x = -u1y; n1y = u1x;
        n2x = -u2y; n2y = u2x;
    } else {
        n1x = u1y; n1y = -u1x;
        n2x = u2y; n2y = -u2x;
    }
    
    // Start point of first line (at end of move1)
    float p1x = move1.xyz[0] + n1x * comp_radius;
    float p1y = move1.xyz[1] + n1y * comp_radius;
    
    // Start point of second line (at start of move2, which is end of move1)
    float p2x = move2.xyz[0] - move2.direction[0] * move2.length_2d + n2x * comp_radius;
    float p2y = move2.xyz[1] - move2.direction[1] * move2.length_2d + n2y * comp_radius;
    
    float det = u1x * u2y - u1y * u2x;
    
    if (fabsf(det) < 0.00001f) {
        // Silent - parallel lines, can't calculate intersection
        return false;
    }
    
    float dx = p2x - p1x;
    float dy = p2y - p1y;
    
    float t1 = (dx * u2y - dy * u2x) / det;
    
    intersection_xy[0] = p1x + t1 * u1x;
    intersection_xy[1] = p1y + t1 * u1y;
    
    bool inside = is_inside_corner(move1, move2);
    
    // Silent - intersection calculated successfully
    
    if (inside && t1 < -0.001f) {
        THEKERNEL->streams->printf("W02:Gouge\n"); // Warning: potential gouge on inside corner
    }
    
    return true;
}

bool CompensationPreprocessor::is_inside_corner(
    const ParsedMove& move1,
    const ParsedMove& move2
) const {
    float cross = cross_product_2d(move1.direction, move2.direction);
    
    bool inside;
    if (comp_side == Compensation::LEFT) {
        inside = (cross < 0);
    } else {
        inside = (cross > 0);
    }
    
    // Silent - corner type determined
    
    return inside;
}

void CompensationPreprocessor::compensate_arc(ParsedMove& arc_move) const {
    if (isnan(arc_move.ijk[0]) && isnan(arc_move.ijk[1])) {
        THEKERNEL->streams->printf("E03:NoIJ\n"); // Error: arc has no I,J offsets
        return;
    }
    
    float i = isnan(arc_move.ijk[0]) ? 0 : arc_move.ijk[0];
    float j = isnan(arc_move.ijk[1]) ? 0 : arc_move.ijk[1];
    float current_radius = sqrtf(i*i + j*j);
    
    if (current_radius < 0.00001f) {
        THEKERNEL->streams->printf("E04:ZeroR\n"); // Error: arc has zero radius
        return;
    }
    
    bool is_cw = (arc_move.mode == MOTION_CW_ARC);
    bool add_radius = (comp_side == Compensation::LEFT && is_cw) || 
                      (comp_side == Compensation::RIGHT && !is_cw);
    
    float new_radius = current_radius + (add_radius ? comp_radius : -comp_radius);
    
    if (new_radius < 0) {
        THEKERNEL->streams->printf("E05:NegR\n"); // Error: arc compensation creates negative radius
        return;
    }
    
    float scale = new_radius / current_radius;
    arc_move.ijk[0] = i * scale;
    arc_move.ijk[1] = j * scale;
    
    // Silent - arc compensated successfully
}

bool CompensationPreprocessor::is_singularity(
    const ParsedMove& move1,
    const ParsedMove& move2
) const {
    if (!is_inside_corner(move1, move2)) {
        return false;
    }
    
    float angle = calculate_angle_between(move1.direction, move2.direction);
    float critical_angle = 2.0f * asinf(comp_radius / (2.0f * comp_radius));
    
    if (angle < critical_angle) {
        THEKERNEL->streams->printf("W03:Singularity\n"); // Warning: singularity detected
        return true;
    }
    
    return false;
}

// ============================================================================
// Geometry Utilities
// ============================================================================

void CompensationPreprocessor::calculate_move_geometry(ParsedMove& move, const float start_pos[2]) {
    float dx = move.xyz[0] - start_pos[0];
    float dy = move.xyz[1] - start_pos[1];
    move.length_2d = sqrtf(dx*dx + dy*dy);
    
    if (move.length_2d > 0.00001f) {
        move.direction[0] = dx / move.length_2d;
        move.direction[1] = dy / move.length_2d;
    } else {
        move.direction[0] = 0;
        move.direction[1] = 0;
    }
}

float CompensationPreprocessor::calculate_angle_between(
    const float dir1[2],
    const float dir2[2]
) const {
    float dot = dir1[0] * dir2[0] + dir1[1] * dir2[1];
    dot = fminf(1.0f, fmaxf(-1.0f, dot));
    return acosf(dot);
}

float CompensationPreprocessor::cross_product_2d(
    const float v1[2],
    const float v2[2]
) const {
    return v1[0] * v2[1] - v1[1] * v2[0];
}

// ============================================================================
// Debug Helpers
// ============================================================================

void CompensationPreprocessor::print_move(const ParsedMove& move, const char* prefix) const {
    // Silent - debug output disabled to prevent buffer overflow
}

void CompensationPreprocessor::print_buffer_state() const {
    // Silent - debug output disabled
}
