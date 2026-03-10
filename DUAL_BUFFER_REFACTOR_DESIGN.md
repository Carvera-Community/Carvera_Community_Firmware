# Dual Buffer Architecture Design
**Date**: March 10, 2026  
**Purpose**: Refactor compensation preprocessor to use two dedicated ring buffers  
**Phase**: Bug fix for Phase 2 + preparation for Phase 3 corner calculations

---

## Architecture Overview

### Current Problem
Single buffer trying to track both uncompensated (programmed) and compensated (offset) coordinates, with confusion about when to update positions (buffering vs output time).

### New Solution
**Two synchronized ring buffers:**
1. **Uncompensated Buffer** (`uncomp_ring`): Source of truth for programmed path
2. **Compensated Buffer** (`comp_ring`): Computed offset path for machine execution

---

## Buffer Structure

### Uncompensated Buffer
```cpp
struct UncompPoint {
    float x, y, z;
};

UncompPoint uncomp_ring[3];  // 3 slots for lookahead
int uncomp_head;              // Write position (next slot to fill)
int uncomp_tail;              // Read position (next slot to output)
int uncomp_count;             // Number of buffered points
```

**Key Properties:**
- Each slot stores **ONE point** (X, Y, Z coordinates)
- Lines are defined by **consecutive slots**: `uncomp_ring[i]` → `uncomp_ring[i+1]`
- Simple storage: just copy coordinates in, no complex logic
- Source of truth for programmed path

### Compensated Buffer
```cpp
struct CompPoint {
    float x, y, z;
    char gcode_string[128];  // Rebuilt Gcode command
};

CompPoint comp_ring[3];      // 3 slots, synchronized with uncomp
int comp_head;                // Write position (locked to uncomp_tail)
int comp_tail;                // Read position (next slot to output)
int comp_count;               // Number of computed compensated points
```

**Key Properties:**
- Each slot stores **ONE compensated point** + rebuilt Gcode
- `comp_head` is **locked** to `uncomp_tail` (synchronization point)
- Computes offset by looking back at last 2-3 slots in `uncomp_ring`
- Outputs actual machine commands

---

## Buffer Synchronization

### Relationship Diagram
```
Uncompensated Buffer (programmed path - fills first):
┌────────┬────────┬────────┐
│ Slot 0 │ Slot 1 │ Slot 2 │
│  A     │  B     │  C     │
└────────┴────────┴────────┘
    ↑               ↑
   tail           head
   
Compensated Buffer (offset path - computes when uncomp outputs):
┌────────┬────────┬────────┐
│ Slot 0 │ Slot 1 │ Slot 2 │
│  A'    │  B'    │  C'    │
└────────┴────────┴────────┘
    ↑               ↑
   tail           head
                   ↑
                   └─ Locked to uncomp_tail
```

### Synchronization Rules
1. **Buffering Phase**: Fill `uncomp_ring` until count=3 (lookahead complete)
2. **Computation Phase**: When `uncomp_count=3`, compute compensated point at `uncomp_tail`
3. **Output Phase**: Output from `comp_tail`, advance both buffers synchronously
4. **Position Tracking**: Implicit - current position is whatever's at the tail positions

---

## Calculation Logic

### Phase 2: Simple Perpendicular Offset

**Input**: Current point at `uncomp_tail`, need to compute compensated version  
**Lookback**: Previous 2 slots (for future corner logic, Phase 2 only uses current line)

```cpp
// Get current point and lookback points
Point curr = uncomp_ring[uncomp_tail];
Point prev1 = uncomp_ring[(uncomp_tail - 1 + 3) % 3];  // Handle wraparound
Point prev2 = uncomp_ring[(uncomp_tail - 2 + 3) % 3];

// Phase 2: Calculate perpendicular offset
// Line direction: prev1 → curr
float dx = curr.x - prev1.x;
float dy = curr.y - prev1.y;
float mag = sqrt(dx*dx + dy*dy);

if (mag < 0.1f) {
    // Zero-length move: look ahead to next point
    // (Special handling needed)
} else {
    // Normalize direction
    dx /= mag;
    dy /= mag;
    
    // Perpendicular normal
    float nx, ny;
    if (compensation_type == LEFT) {
        nx = -dy;  // 90° CCW
        ny = dx;
    } else {
        nx = dy;   // 90° CW
        ny = -dx;
    }
    
    // Apply offset to current point
    comp_ring[comp_head].x = curr.x + nx * radius;
    comp_ring[comp_head].y = curr.y + ny * radius;
    comp_ring[comp_head].z = curr.z;
    
    // Rebuild Gcode
    sprintf(comp_ring[comp_head].gcode_string, "G1 X%.3f Y%.3f Z%.3f", ...);
}
```

### Phase 3: Corner Intersections (Future)

```cpp
// To calculate corner at point B (current output position):
// Look at line A→B and line B→C

Point A = uncomp_ring[(uncomp_tail - 1 + 3) % 3];  // Previous point
Point B = uncomp_ring[uncomp_tail];                 // Current point
Point C = uncomp_ring[(uncomp_tail + 1) % 3];      // Next point (still buffered)

// Calculate offset line 1: A'→B' (parallel to A→B)
// Calculate offset line 2: B'→C' (parallel to B→C)
// Find intersection of these two offset lines
// Result becomes compensated point B'
```

---

## Special Cases

### Program Start (First 1-2 Lines)

**Problem**: Lookback needs 2 previous points, but buffer starts empty.

**Solution**:
- **First line** (uncomp_count=1): No previous line exists
  - Output: Simply offset the first point perpendicular to... what direction?
  - **Decision needed**: Do we use direction to point 2? Or just offset based on current position?
  - User said: "compensated 'starting point' simply starts at the offset position. This creates an extra movement at the beginning but that is fine."
  - **Action**: Output lead-in move to offset position (0,0) → (offset 0,0)
  
- **Second line** (uncomp_count=2): One previous point exists
  - Can calculate direction: point[0] → point[1]
  - Apply perpendicular offset as normal
  
- **Third+ lines** (uncomp_count=3): Full buffer, normal operation
  - Can look back 2 slots for corner calculations (Phase 3)

### Zero-Length Moves

**Problem**: `G1 X50 Y50 Z10` (Z-only move, no XY change) has no direction.

**Current Approach**: Look ahead to next move for direction

**New Approach with Dual Buffers**:
```cpp
if (mag < 0.1f) {
    // Look ahead in uncomp_ring
    Point next = uncomp_ring[(uncomp_tail + 1) % 3];
    
    // Use direction from curr → next
    // Or if still zero-length, keep looking ahead
    // Or as last resort, use previous direction
}
```

**Storage**: Always store in `uncomp_ring` even if zero-length (user confirmed)

### G40 Passthrough Mode

When compensation is OFF:
- Input Gcode bypasses both buffers (current behavior)
- Buffers remain empty/clear
- Direct output to motion controller

---

## Variable Naming Schema

### Ring Buffers
```cpp
// Uncompensated buffer (programmed path)
UncompPoint uncomp_ring[3];
int uncomp_head;   // Next slot to write
int uncomp_tail;   // Next slot to output/read
int uncomp_count;  // Slots occupied

// Compensated buffer (offset path)
CompPoint comp_ring[3];
int comp_head;     // Next slot to write (locked to uncomp_tail)
int comp_tail;     // Next slot to output
int comp_count;    // Slots occupied
```

### Working Variables (during calculation)
```cpp
// Lookback cache (during offset calculation)
Point uncomp_curr;   // Current point being processed (at uncomp_tail)
Point uncomp_prev1;  // Previous point (1 slot back)
Point uncomp_prev2;  // Previous point (2 slots back)

// Direction and normal vectors
float dir_x, dir_y;     // Normalized direction of current line
float norm_x, norm_y;   // Perpendicular normal vector

// Offset result
float comp_x, comp_y, comp_z;  // Computed compensated coordinates
```

### Configuration
```cpp
float comp_radius;        // Compensation radius (tool radius)
CompensationType comp_type;  // LEFT or RIGHT (retain current enum)
bool comp_active;         // Is G41/G42 active (vs G40 passthrough)
```

---

## Algorithm Flow

### Main Processing Functions

#### 1. `on_gcode_received()` - Entry point
```cpp
void on_gcode_received(Gcode* gcode) {
    // Handle G40/G41/G42 mode switches
    if (gcode->has_g && gcode->g == 40) {
        flush_buffers();
        comp_active = false;
        return;
    }
    
    if (gcode->has_g && (gcode->g == 41 || gcode->g == 42)) {
        comp_type = (gcode->g == 41) ? LEFT : RIGHT;
        comp_radius = ... // Get D word
        comp_active = true;
        return;
    }
    
    // Route based on mode
    if (!comp_active) {
        // Passthrough mode - direct output
        THEKERNEL->conveyor->append_gcode(gcode);
    } else {
        // Compensation active - buffer it
        buffer_gcode(gcode);
    }
}
```

#### 2. `buffer_gcode()` - Add to uncompensated buffer
```cpp
void buffer_gcode(Gcode* gcode) {
    // Extract coordinates
    float x = gcode->has_letter('X') ? gcode->get_value('X') : uncomp_ring[(uncomp_head-1+3)%3].x;
    float y = gcode->has_letter('Y') ? gcode->get_value('Y') : uncomp_ring[(uncomp_head-1+3)%3].y;
    float z = gcode->has_letter('Z') ? gcode->get_value('Z') : uncomp_ring[(uncomp_head-1+3)%3].z;
    
    // Store in uncomp buffer
    uncomp_ring[uncomp_head].x = x;
    uncomp_ring[uncomp_head].y = y;
    uncomp_ring[uncomp_head].z = z;
    
    uncomp_head = (uncomp_head + 1) % 3;
    uncomp_count++;
    
    print_debug_uncomp_buffered();
    
    // If buffer full, start processing
    if (uncomp_count >= 3) {
        compute_and_output();
    }
}
```

#### 3. `compute_and_output()` - Calculate offset and output
```cpp
void compute_and_output() {
    // Cache lookback points
    uncomp_curr = uncomp_ring[uncomp_tail];
    uncomp_prev1 = uncomp_ring[(uncomp_tail - 1 + 3) % 3];
    uncomp_prev2 = uncomp_ring[(uncomp_tail - 2 + 3) % 3];  // For Phase 3
    
    // Calculate direction: prev1 → curr
    float dx = uncomp_curr.x - uncomp_prev1.x;
    float dy = uncomp_curr.y - uncomp_prev1.y;
    float mag = sqrt(dx*dx + dy*dy);
    
    if (mag < 0.1f) {
        handle_zero_length_move();
        return;
    }
    
    // Normalize and perpendicular
    dir_x = dx / mag;
    dir_y = dy / mag;
    
    if (comp_type == LEFT) {
        norm_x = -dir_y;
        norm_y = dir_x;
    } else {
        norm_x = dir_y;
        norm_y = -dir_x;
    }
    
    // Apply offset
    comp_x = uncomp_curr.x + norm_x * comp_radius;
    comp_y = uncomp_curr.y + norm_y * comp_radius;
    comp_z = uncomp_curr.z;
    
    // Store in comp buffer
    comp_ring[comp_head].x = comp_x;
    comp_ring[comp_head].y = comp_y;
    comp_ring[comp_head].z = comp_z;
    
    // Rebuild Gcode
    sprintf(comp_ring[comp_head].gcode_string, "G1 X%.3f Y%.3f Z%.3f", comp_x, comp_y, comp_z);
    
    comp_head = (comp_head + 1) % 3;
    comp_count++;
    
    print_debug_offset_calc();
    
    // Output from comp buffer
    output_compensated();
}
```

#### 4. `output_compensated()` - Send to machine
```cpp
void output_compensated() {
    if (comp_count == 0) return;
    
    // Get compensated Gcode from tail
    CompPoint* pt = &comp_ring[comp_tail];
    
    // Create new Gcode object from string
    Gcode gc(pt->gcode_string, ...);
    
    print_debug_output();
    
    // Send to conveyor
    THEKERNEL->conveyor->append_gcode(&gc);
    
    // Advance both buffers synchronously
    comp_tail = (comp_tail + 1) % 3;
    comp_count--;
    
    uncomp_tail = (uncomp_tail + 1) % 3;
    uncomp_count--;
}
```

#### 5. `flush_buffers()` - End of program or G40
```cpp
void flush_buffers() {
    // Process all remaining buffered points
    while (uncomp_count > 0) {
        compute_and_output();
    }
    
    // Clear state
    uncomp_head = uncomp_tail = uncomp_count = 0;
    comp_head = comp_tail = comp_count = 0;
}
```

---

## Debug Output Strategy

### Debug Print Functions

```cpp
void print_debug_uncomp_buffered() {
    // Show uncomp buffer state after adding point
    printf(">>UNCOMP_BUFFER: count=%d head=%d tail=%d\n", uncomp_count, uncomp_head, uncomp_tail);
    printf("  Slot[%d]: (%.3f, %.3f, %.3f)\n", uncomp_tail, uncomp_ring[uncomp_tail].x, ...);
}

void print_debug_offset_calc() {
    // Show offset calculation details
    printf(">>OFFSET_CALC:\n");
    printf("  Uncomp: (%.3f, %.3f) -> (%.3f, %.3f)\n", uncomp_prev1.x, uncomp_prev1.y, uncomp_curr.x, uncomp_curr.y);
    printf("  Direction: (%.3f, %.3f)\n", dir_x, dir_y);
    printf("  Normal: (%.3f, %.3f)\n", norm_x, norm_y);
    printf("  Compensated: (%.3f, %.3f)\n", comp_x, comp_y);
}

void print_debug_output() {
    // Show what's being output
    printf(">>OUTPUT: '%s'\n", comp_ring[comp_tail].gcode_string);
}

void print_debug_buffer_state() {
    // Show both buffers
    printf(">>BUFFERS: uncomp=%d/%d comp=%d/%d\n", uncomp_count, uncomp_head, comp_count, comp_head);
}
```

---

## Implementation Checklist

### Phase 1: Data Structures (CompensationPreprocessor.h)
- [ ] Define `UncompPoint` struct
- [ ] Define `CompPoint` struct  
- [ ] Add `uncomp_ring[3]`, `uncomp_head`, `uncomp_tail`, `uncomp_count`
- [ ] Add `comp_ring[3]`, `comp_head`, `comp_tail`, `comp_count`
- [ ] Rename `compensation_type` → `comp_type`
- [ ] Rename `compensation_radius` → `comp_radius`
- [ ] Add `comp_active` flag
- [ ] Remove old `BufferedGcode` struct and related variables
- [ ] Remove old `uncompensated_position[3]`, `compensated_position[3]` arrays

### Phase 2: Core Functions (CompensationPreprocessor.cpp)
- [ ] Rewrite `on_gcode_received()` with new routing logic
- [ ] Implement `buffer_gcode()` - simple coordinate extraction and storage
- [ ] Implement `compute_and_output()` - offset calculation using lookback
- [ ] Implement `output_compensated()` - send to conveyor and advance buffers
- [ ] Update `flush_buffers()` for new architecture
- [ ] Remove old `clone_and_extract()` function
- [ ] Remove old `get_compensated_gcode()` function

### Phase 3: Special Cases
- [ ] Handle program start (first 1-2 lines) with limited lookback
- [ ] Handle zero-length moves with lookahead
- [ ] Ensure G40 passthrough works correctly

### Phase 4: Debug Output
- [ ] Update all debug print functions for new variable names
- [ ] Add new debug for uncomp/comp buffer states
- [ ] Ensure debug output shows synchronization clearly

### Phase 5: Testing
- [ ] Build firmware
- [ ] Test `TEST_D_word_simple_square_in_square.cnc`
- [ ] Verify no diagonal movements
- [ ] Verify position tracking is implicit and synchronized
- [ ] Verify corner gaps present (expected for Phase 2)

---

## Expected Outcomes

### Phase 2 Success Criteria
✅ No diagonal movements during straight-line segments  
✅ Perpendicular offset applied correctly  
✅ Position tracking synchronized (implicit in buffer positions)  
✅ Debug output shows clear uncomp → comp transformation  
✅ Corner gaps visible (expected, will fix in Phase 3)  

### Benefits of New Architecture
- **Clarity**: Two buffers, two purposes - no confusion
- **Correctness**: Synchronization explicit, no race conditions
- **Extensibility**: Easy to add corner calculations (look back at 3 points)
- **Simplicity**: No complex position update logic, just buffer management

---

## Questions for Review

1. **Program start behavior**: For the first point, should we output a lead-in move from current position to offset position? Or start compensated path directly?

2. **Buffer size**: Confirmed 3 slots for both. Any performance concerns?

3. **Zero-length move direction**: If zero-length followed by another zero-length, how far ahead should we look for direction?

4. **Debug verbosity**: Current debug output is very detailed. Keep it for Phase 2, or reduce?

5. **Gcode rebuild**: Should `comp_ring` store the full Gcode string, or just coordinates + rebuild on output?

---

**Ready to proceed with implementation?** Please review and let me know if any changes needed!
