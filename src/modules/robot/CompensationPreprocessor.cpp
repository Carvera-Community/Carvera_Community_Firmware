#include "CompensationPreprocessor.h"
#include "Robot.h"
#include "libs/StreamOutputPool.h"
#include "libs/Kernel.h"
#include <cmath>

CompensationPreprocessor::CompensationPreprocessor() 
     : comp_side(Compensation::NONE), comp_radius(0) {
    THEKERNEL->streams->printf("DBG:CompPrep: Preprocessor initialized\n");
}

void CompensationPreprocessor::enable_compensation(CompSide side, float diameter) {
    // Validate input
    if (side != Compensation::NONE && side != Compensation::LEFT && side != Compensation::RIGHT) {
        THEKERNEL->streams->printf("ERROR:CompPrep: Invalid compensation side %d requested\n", side);
        return;
    }

    THEKERNEL->streams->printf("DBG:CompPrep: Enabling compensation side=%d diameter=%.3f\n", side, diameter);
    comp_side = side;
    comp_radius = diameter / 2.0f;
    move_buffer.clear();
    THEKERNEL->streams->printf("DBG:CompPrep: Enabled %s compensation, radius=%.3f\n", 
         side == Compensation::LEFT ? "LEFT" : "RIGHT", comp_radius);
}

void CompensationPreprocessor::disable_compensation() {
     if (comp_side != Compensation::NONE) {
        flush_moves();
        comp_side = Compensation::NONE;
        comp_radius = 0;
        THEKERNEL->streams->printf("DBG:CompPrep: Compensation disabled\n");
    }
}

bool CompensationPreprocessor::preprocess_move(Gcode* gcode, float* target, float* position) {
    // Validate compensation state
    if (comp_side != Compensation::NONE && comp_side != Compensation::LEFT && comp_side != Compensation::RIGHT) {
        THEKERNEL->streams->printf("ERROR:CompPrep: Invalid compensation side %d\n", comp_side);
        comp_side = Compensation::NONE;
        return false;
    }

    THEKERNEL->streams->printf("DBG:CompPrep: Processing move [%.3f,%.3f] -> [%.3f,%.3f] (comp_side=%d)\n",
        position[0], position[1], target[0], target[1], comp_side);
        
    if (comp_side == Compensation::NONE) {
        return false;  // Pass through if compensation not active
    }
    Move move;
    move.start[0] = position[0];
    move.start[1] = position[1];
    move.end[0] = target[0];
    move.end[1] = target[1];
    move.is_arc = false;  // TODO: Handle arcs
     // Store line number if available from M code
     move.line_number = (gcode->m > 0) ? gcode->m : 0;    // Buffer the move
    buffer_move(move);
    
    // Need at least 2 moves to start processing
    if (move_buffer.size() < 2) {
        return true;  // Consumed but not ready to output
    }
    
    // Process oldest move in buffer if we have enough context
    Move& current = move_buffer[0];
    float output[2];
    
    if (move_buffer.size() >= 3) {
        // We have previous, current, and next moves - handle corners
        calculate_intersection(move_buffer[0], move_buffer[1], output);
    } else {
        // Simple perpendicular offset for the move
        calculate_line_offset(current, output);
    }
    
    // Update target with compensated position
    target[0] = output[0];
    target[1] = output[1];
    
    THEKERNEL->streams->printf("DBG:CompPrep: Move #%d offset [%.3f,%.3f] -> [%.3f,%.3f]\n",
        current.line_number, current.end[0], current.end[1], output[0], output[1]);
    
    return true;
}

void CompensationPreprocessor::calculate_line_offset(const Move& move, float* output) {
    // Calculate move vector
    float dx = move.end[0] - move.start[0];
    float dy = move.end[1] - move.start[1];
    float len = hypotf(dx, dy);
    
    if (len < 0.00001F) {
        output[0] = move.end[0];
        output[1] = move.end[1];
        THEKERNEL->streams->printf("DBG:CompPrep: Zero length move skipped\n");
        return;
    }
    
    // Calculate unit vectors
    float ux = dx / len;  // Unit vector along move
    float uy = dy / len;
    
    // Calculate normal vector (perpendicular)
    float nx, ny;
    if (comp_side == Compensation::LEFT) {
        nx = -uy;  // Rotate 90° CCW
        ny = ux;
    } else {
        nx = uy;   // Rotate 90° CW
        ny = -ux;
    }
    
    // Apply offset in workpiece coordinates
    output[0] = move.end[0] + nx * comp_radius;
    output[1] = move.end[1] + ny * comp_radius;
    
    THEKERNEL->streams->printf("DBG:CompPrep: Move vector [%.3f,%.3f] normal [%.3f,%.3f]\n", 
        ux, uy, nx, ny);
}

void CompensationPreprocessor::buffer_move(const Move& move) {
    move_buffer.push_back(move);
    print_move(move, "Buffered");
    
    // Keep buffer size limited
    while (move_buffer.size() > LOOKAHEAD_SIZE) {
        move_buffer.erase(move_buffer.begin());
    }
}

void CompensationPreprocessor::print_move(const Move& move, const char* prefix) const {
    THEKERNEL->streams->printf("DBG:CompPrep: %s move #%d [%.3f,%.3f] -> [%.3f,%.3f]\n",
        prefix, move.line_number, 
        move.start[0], move.start[1],
        move.end[0], move.end[1]);
    if (move.is_arc) {
        THEKERNEL->streams->printf("DBG:CompPrep: Arc center [%.3f,%.3f] radius=%.3f %s\n",
            move.center[0], move.center[1], move.radius,
            move.clockwise ? "CW" : "CCW");
    }
}

void CompensationPreprocessor::flush_moves() {
    move_buffer.clear();
}

void CompensationPreprocessor::preprocess_arc_offsets(float offset[2], bool clockwise) {
     if (comp_side == Compensation::NONE) return;    // For arcs, we adjust the radius by modifying the I,J offsets
    // - For G41 (left) and G2 (CW) or G42 (right) and G3 (CCW): add radius
    // - For G41 (left) and G3 (CCW) or G42 (right) and G2 (CW): subtract radius
     bool add_radius = (comp_side == Compensation::LEFT && clockwise) || (comp_side == Compensation::RIGHT && !clockwise);
    float radius_adjustment = add_radius ? comp_radius : -comp_radius;

    // Calculate current radius from I,J
    float current_radius = hypotf(offset[0], offset[1]);
    if (current_radius < 0.00001F) return;  // Skip if radius too small

    // Adjust I,J proportionally to change radius
    float scale = (current_radius + radius_adjustment) / current_radius;
    offset[0] *= scale;
    offset[1] *= scale;

    THEKERNEL->streams->printf("DBG:CompPrep: Arc offset adjusted from r=%.3f to r=%.3f\n",
        current_radius, current_radius + radius_adjustment);
}