# Cutter Compensation Implementation Strategy

## Overview
Implementation of cutter compensation (G41/G42) following LinuxCNC semantics, with direct arc offset calculations and integration with existing tool measurement capabilities (M491).

## Data Storage
Extend existing EEPROM_data structure to store tool diameter alongside current TLO storage:
```cpp
typedef struct {
    // ... existing fields ...
    float TLO;         // Tool Length Offset
    float TOOL_DIA;    // Tool Diameter for compensation
    // ... remaining fields ...
} EEPROM_data;
```

## G-Code Interface
- `G41` - Enable cutter compensation left of path
- `G42` - Enable cutter compensation right of path
- `G40` - Cancel cutter compensation
- Use M491 for tool measurement and diameter capture

## Runtime State in Robot
```cpp
class Robot {
    struct {
        bool cutter_comp_on:1;
        bool comp_left:1;     // true=G41, false=G42
    };
    float cutter_radius;      // Half of TOOL_DIA
};
```

## Core Mathematics
### Linear Moves
```cpp
// After computing target[] in MCS:
void apply_linear_compensation(float *target) {
    float dx = target[X_AXIS] - machine_position[X_AXIS];
    float dy = target[Y_AXIS] - machine_position[Y_AXIS];
    float len = sqrt(dx*dx + dy*dy);
    if(len < 0.00001F) return;  // Skip tiny moves
    
    // Unit normal vector (-dy/len, dx/len) points left of direction
    float nx = -dy/len;
    float ny = dx/len;
    float offset = (comp_left ? 1 : -1) * cutter_radius;
    
    target[X_AXIS] += offset * nx;
    target[Y_AXIS] += offset * ny;
}
```

### Arc Moves
```cpp
void apply_arc_compensation(float *target, float *offset, bool clockwise) {
    // Compute new arc center by offsetting perpendicular to radius
    float radius = sqrt(offset[0]*offset[0] + offset[1]*offset[1]);
    float direction = (comp_left == clockwise) ? 1 : -1;
    float new_radius = radius + direction * cutter_radius;
    
    // Scale I,J offset to new radius
    float scale = new_radius / radius;
    offset[0] *= scale;
    offset[1] *= scale;
}
```

## Integration Points

### 1. G-Code Parsing
Add to Robot::on_gcode_received():
```cpp
case 40: cutter_comp_on = false; break;
case 41: cutter_comp_on = true; comp_left = true; break;
case 42: cutter_comp_on = true; comp_left = false; break;
```

### 2. Move Processing
Modify Robot::process_move() to apply compensation after WCS transforms but before compensationTransform:
```cpp
// After computing target[] in MCS
if(cutter_comp_on) {
    switch(motion_mode) {
        case LINEAR: apply_linear_compensation(target); break;
        case CW_ARC:
        case CCW_ARC: apply_arc_compensation(target, offset, motion_mode == CW_ARC); break;
    }
}
```

### 3. Tool Measurement
Extend M491 handler to store measured diameter in EEPROM:
```cpp
// After successful probe measurement:
THEKERNEL->eeprom_data->TOOL_DIA = measured_diameter;
THEKERNEL->write_eeprom_data();
robot->cutter_radius = measured_diameter * 0.5F;
```

## Testing Strategy

### Unit Tests
1. Linear Compensation
   ```cpp
   void test_linear_comp() {
       Robot r;
       r.cutter_comp_on = true;
       r.comp_left = true;
       r.cutter_radius = 2.0f;
       
       float target[3] = {100, 0, 0};  // X100 Y0 move
       r.apply_linear_compensation(target);
       
       assert(fabs(target[X_AXIS] - 100) < 0.001f);
       assert(fabs(target[Y_AXIS] - 2.0f) < 0.001f);
   }
   ```

2. Arc Compensation
   ```cpp
   void test_arc_comp() {
       Robot r;
       r.cutter_comp_on = true;
       r.comp_left = true;
       r.cutter_radius = 2.0f;
       
       float target[3] = {10, 10, 0};
       float offset[2] = {10, 0};  // 10mm radius CW arc
       r.apply_arc_compensation(target, offset, true);
       
       assert(fabs(sqrt(offset[0]*offset[0] + offset[1]*offset[1]) - 12.0f) < 0.001f);
   }
   ```

### Manual Validation
Test G-code sequence:
```gcode
G90 G54      ; Absolute coordinates
M491         ; Measure tool diameter
G0 X0 Y0
G41          ; Compensation left
G1 X50 F1000 ; Should offset Y+diameter/2
G2 X100 Y50 I50 J0 ; Should increase arc radius
G1 Y100      ; Should offset X-diameter/2
G40          ; Cancel compensation
M2
```

## Implementation Phases

### Phase 1: Core Framework
- Add cutter compensation fields to Robot
- Implement G40/G41/G42 parsing
- Add linear and arc offset math
- Basic validation of offset calculations

### Phase 2: Measurement Integration  
- Extend M491 to store diameter in EEPROM
- Add persistence logic
- Implement status reporting in query strings

### Phase 3: Testing & Validation
- Unit test implementation
- Example G-code validation suite
- Edge case testing (tiny moves, full circles)