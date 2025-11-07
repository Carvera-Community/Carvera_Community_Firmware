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

    // Calculate move vector
    float dx = target[0] - position[0];
    float dy = target[1] - position[1];
    float move_length = sqrtf(dx*dx + dy*dy);

    // For very short moves, maintain the same offset direction as the previous move
    if (move_length < 0.1f && !move_buffer.empty()) {
        const Move& prev = move_buffer.back();
        target[0] = position[0] + dx + prev.direction[1] * comp_radius;
        target[1] = position[1] + dy - prev.direction[0] * comp_radius;
        
        THEKERNEL->streams->printf("DBG:CompPrep: Short move (%.3fmm), maintaining direction\n", move_length);
        return true;
    }
    
    Move move;
    move.start[0] = position[0];
    move.start[1] = position[1];
    move.end[0] = target[0];
    move.end[1] = target[1];
    move.is_arc = false;  // TODO: Handle arcs
    move.line_number = (gcode->m > 0) ? gcode->m : 0;
    
    // Calculate and cache move properties
    float dx = target[0] - position[0];
    float dy = target[1] - position[1];
    move.length = sqrtf(dx*dx + dy*dy);
    
    if (move.length > 0.00001F) {
        move.direction[0] = dx / move.length;  // Unit vector X
        move.direction[1] = dy / move.length;  // Unit vector Y
    } else {
        // For zero-length moves, use previous direction or default to X axis
        if (!move_buffer.empty()) {
            move.direction[0] = move_buffer.back().direction[0];
            move.direction[1] = move_buffer.back().direction[1];
        } else {
            move.direction[0] = 1.0F;
            move.direction[1] = 0.0F;
        }
    }
    buffer_move(move);
    
    // Process current move immediately for straight lines
    float output[2];
    calculate_line_offset(move, output);
    
    // Update the output position
    target[0] = output[0];
    target[1] = output[1];
    
    THEKERNEL->streams->printf("DBG:CompPrep: Applied offset [%.3f,%.3f] -> [%.3f,%.3f]\n", 
        move.end[0], move.end[1], output[0], output[1]);
    
    // Remove old moves from buffer if needed
    if (move_buffer.size() > 5) {
        move_buffer.erase(move_buffer.begin());
    }
    
    return true;
}

void CompensationPreprocessor::calculate_line_offset(const Move& move, float* output) {
    // Use cached direction vector
    float ux = move.direction[0];
    float uy = move.direction[1];
    
    // Calculate normal vector based on compensation side
    float nx, ny;
    if (comp_side == Compensation::LEFT) {
        nx = -uy;  // Rotate 90° CCW
        ny = ux;
    } else {
        nx = uy;   // Rotate 90° CW
        ny = -ux;
    }
    
    // Apply offset to both start and end points
    output[0] = move.end[0] + nx * comp_radius;
    output[1] = move.end[1] + ny * comp_radius;
    
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