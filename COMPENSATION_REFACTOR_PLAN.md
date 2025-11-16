# Cutter Compensation Refactor Plan
## Memory-Efficient True Lookahead Implementation

### Executive Summary
Refactor the cutter compensation system to use **true lookahead buffering** with minimal memory overhead by storing only parsed move data instead of full G-code objects.

---

## Architecture Overview

### Data Flow
```
G-code Stream Input
    ↓
Robot::on_gcode_received()
    ↓ Parse G-code to ParsedMove (using existing Robot parsing)
    ↓
CompensationPreprocessor::buffer_move()
    ↓ Buffer until LOOKAHEAD_DEPTH (3) moves
    ↓
CompensationPreprocessor::get_compensated_move()
    ↓ Apply compensation with full corner visibility
    ↓
Robot::process_move()
    ↓ Apply WCS transforms
    ↓
append_line() / append_arc()
    ↓ Segmentation
    ↓
Planner / Stepper (UNCHANGED)
```

---

## Memory Optimization Strategy

### Current Approach Problems:
- Storing full `Gcode` objects: ~300+ bytes each
- String storage: variable, can be large
- 3-move buffer = ~1KB+ memory

### New Approach Benefits:
- `ParsedMove` struct: ~80 bytes each
- No string storage
- 3-move buffer = ~240 bytes
- **75% memory reduction!**

### ParsedMove Structure (80 bytes):
```cpp
struct ParsedMove {
    float xyz[3];          // 12 bytes - endpoint coordinates
    float ijk[3];          // 12 bytes - arc offsets
    float direction[2];    //  8 bytes - cached unit vector
    float length_2d;       //  4 bytes - cached 2D distance
    float feed_rate;       //  4 bytes - F parameter
    unsigned int line_num; //  4 bytes - for debugging
    MotionMode mode;       //  4 bytes - enum (G0/G1/G2/G3)
    bool valid;            //  1 byte  - validity flag
                           // +31 bytes padding/alignment
                           // = ~80 bytes total
};
```

---

## Implementation Steps

### Phase 1: Update CompensationPreprocessor.h ✅ COMPLETE
- [x] Define `ParsedMove` struct
- [x] Define `MotionMode` enum
- [x] Declare buffer management functions
- [x] Declare core compensation algorithms
- [x] Add geometry utility functions

### Phase 2: Implement Robot.cpp Integration (NEXT)

#### Step 2.1: Add Helper Function to Parse Moves
```cpp
// In Robot.cpp - NEW FUNCTION
CompensationPreprocessor::ParsedMove Robot::parse_move_for_compensation(
    Gcode* gcode, 
    enum MOTION_MODE_T motion_mode
) {
    CompensationPreprocessor::ParsedMove parsed;
    
    // Convert motion mode enum
    switch(motion_mode) {
        case SEEK:    parsed.mode = CompensationPreprocessor::MOTION_SEEK; break;
        case LINEAR:  parsed.mode = CompensationPreprocessor::MOTION_LINEAR; break;
        case CW_ARC:  parsed.mode = CompensationPreprocessor::MOTION_CW_ARC; break;
        case CCW_ARC: parsed.mode = CompensationPreprocessor::MOTION_CCW_ARC; break;
        default:      parsed.mode = CompensationPreprocessor::MOTION_NONE; break;
    }
    
    // Parse XYZ using existing Robot parsing logic
    for(int i= X_AXIS; i <= Z_AXIS; ++i) {
        char letter= 'X'+i;
        if( gcode->has_letter(letter) ) {
            parsed.xyz[i] = this->to_millimeters(gcode->get_value(letter));
            
            // Convert incremental to absolute
            if (!this->absolute_mode) {
                parsed.xyz[i] += last_gcode_position[i];
            }
        } else {
            parsed.xyz[i] = last_gcode_position[i];  // Inherit previous position
        }
    }
    
    // Parse IJK for arcs
    if(motion_mode == CW_ARC || motion_mode == CCW_ARC) {
        for(char letter = 'I'; letter <= 'K'; letter++) {
            int idx = letter - 'I';
            if( gcode->has_letter(letter) ) {
                parsed.ijk[idx] = this->to_millimeters(gcode->get_value(letter));
            } else {
                parsed.ijk[idx] = NAN;
            }
        }
    }
    
    // Parse feed rate
    if( gcode->has_letter('F') ) {
        parsed.feed_rate = this->to_millimeters(gcode->get_value('F'));
    } else {
        parsed.feed_rate = NAN;  // Use modal feed rate
    }
    
    // Store line number
    parsed.line_num = gcode->line;
    parsed.valid = true;
    
    return parsed;
}
```

#### Step 2.2: Modify on_gcode_received()
```cpp
void Robot::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);
    enum MOTION_MODE_T motion_mode= NONE;

    // Handle G-code parsing (UNCHANGED)
    if( gcode->has_g) {
        switch( gcode->g ) {
            case 0:  motion_mode = SEEK;    break;
            case 1:  motion_mode = LINEAR;  break;
            case 2:  motion_mode = CW_ARC;  break;
            case 3:  motion_mode = CCW_ARC; break;
            
            case 40: // G40: Cancel cutter compensation
                set_compensation(Compensation::NONE, 0.0f);
                
                // Flush buffered moves FIRST
                std::queue<CompensationPreprocessor::ParsedMove> flushed;
                comp_preprocessor->flush_buffer(flushed);
                
                // Process all flushed moves
                while(!flushed.empty()) {
                    process_parsed_move(flushed.front());
                    flushed.pop();
                }
                return;
                
            case 41: // G41: Enable left compensation
            case 42: // G42: Enable right compensation
                // ... (handle D word, enable compensation)
                return;
        }
    }

    // **NEW: If compensation active and this is a movement command**
    if(comp_preprocessor->is_active() && motion_mode != NONE) {
        // Parse the move into lightweight structure
        CompensationPreprocessor::ParsedMove parsed = parse_move_for_compensation(gcode, motion_mode);
        
        // Buffer it
        comp_preprocessor->buffer_move(parsed);
        
        // Try to get a compensated move from the buffer
        CompensationPreprocessor::ParsedMove compensated;
        if(comp_preprocessor->get_compensated_move(compensated)) {
            // Process the compensated move
            process_parsed_move(compensated);
        }
        // If buffer doesn't have enough moves yet, we're done (latency building)
        return;
    }
    
    // **NORMAL PATH: No compensation or non-movement command**
    if(motion_mode != NONE) {
        process_move(gcode, motion_mode);
    }
}
```

#### Step 2.3: Create process_parsed_move()
```cpp
// NEW FUNCTION to process a ParsedMove
void Robot::process_parsed_move(const CompensationPreprocessor::ParsedMove& parsed)
{
    // Convert ParsedMove back to the format process_move() expects
    
    // Create target array in machine coordinates
    float target[n_motors];
    memcpy(target, machine_position, n_motors*sizeof(float));
    
    // Apply WCS transforms to the (already compensated) coordinates
    for(int i= X_AXIS; i <= Z_AXIS; ++i) {
        if(!isnan(parsed.xyz[i])) {
            target[i] = ROUND_NEAR_HALF(
                parsed.xyz[i] + 
                std::get<i>(wcs_offsets[current_wcs]) - 
                std::get<i>(g92_offset) + 
                std::get<i>(tool_offset)
            );
        }
    }
    
    // Handle feed rate
    float rate_mm_s;
    if(!isnan(parsed.feed_rate)) {
        if(parsed.mode == CompensationPreprocessor::MOTION_SEEK) {
            this->seek_rate = parsed.feed_rate;
        } else {
            this->feed_rate = parsed.feed_rate;
        }
    }
    
    if(parsed.mode == CompensationPreprocessor::MOTION_SEEK) {
        rate_mm_s = this->seek_rate / seconds_per_minute;
    } else {
        rate_mm_s = this->feed_rate / seconds_per_minute;
    }
    
    // Execute the move
    bool moved = false;
    switch(parsed.mode) {
        case CompensationPreprocessor::MOTION_SEEK:
        case CompensationPreprocessor::MOTION_LINEAR:
            moved = this->append_line(nullptr, target, rate_mm_s, NAN);
            break;
            
        case CompensationPreprocessor::MOTION_CW_ARC:
        case CompensationPreprocessor::MOTION_CCW_ARC:
            // Extract arc offsets from ParsedMove
            float offset[3];
            offset[0] = parsed.ijk[0];
            offset[1] = parsed.ijk[1];
            offset[2] = parsed.ijk[2];
            moved = this->compute_arc(nullptr, offset, target, 
                parsed.mode == CompensationPreprocessor::MOTION_CW_ARC ? CW_ARC : CCW_ARC);
            break;
    }
    
    if(moved) {
        memcpy(machine_position, target, n_motors * sizeof(float));
    }
}
```

### Phase 3: Implement CompensationPreprocessor.cpp

#### Key Functions to Implement:

1. **buffer_move()** - Add move to queue
2. **get_compensated_move()** - Calculate compensation with lookahead
3. **calculate_perpendicular_offset()** - Simple line offset
4. **calculate_corner_intersection()** - Corner handling
5. **is_inside_corner()** - Detect corner type
6. **compensate_arc()** - Arc I,J,K modification
7. **flush_buffer()** - Handle program end/G40

---

## Testing Plan

### Test 1: Basic Line with Compensation
```gcode
G90 G54
G0 X0 Y0 Z5
G41 D6          ; First 3 moves get buffered
G1 F1000 X10 Y0 ; Move 1: buffered
G1 X20 Y0       ; Move 2: buffered
G1 X30 Y0       ; Move 3: buffered, Move 1 now executes with compensation
G40
M2
```

### Test 2: Outside Corner
```gcode
G90 G54
G0 X0 Y0 Z5
G41 D6
G1 F1000 X50 Y0
G1 X50 Y50      ; 90° outside corner
G40
M2
```

### Test 3: Inside Corner
```gcode
G90 G54
G0 X50 Y50 Z5
G41 D6
G1 F1000 X0 Y50
G1 X0 Y0        ; 90° inside corner
G40
M2
```

---

## Advantages of This Approach

### Memory Efficiency
- **75% reduction** in buffer memory
- Scales well with larger lookahead depths
- No string parsing/storage overhead

### Performance
- Parse once using existing Robot functions
- Cache geometry calculations (length, direction)
- Clean separation of parsing vs. compensation logic

### Maintainability
- Robot continues to handle all parsing
- Preprocessor focuses purely on geometry
- Clear API boundary

### Extensibility
- Easy to add more lookahead depth
- Can add singularity detection
- Can insert lead-in/lead-out moves
- Future: detect and simplify collinear moves

---

## Next Steps

1. Implement CompensationPreprocessor::buffer_move()
2. Implement CompensationPreprocessor::get_compensated_move()
3. Test with simple single line (no corners)
4. Add corner intersection logic
5. Test with outside corners
6. Test with inside corners
7. Add arc support
8. Add singularity detection

---

## Questions Before Implementation

1. Should G0 (rapid) moves use compensation or turn it off temporarily?
2. What should happen on M2 (program end) if moves are still buffered?
3. Should we warn the user about the 3-move latency on G41/G42?
4. What's the maximum acceptable inside corner angle before singularity?

