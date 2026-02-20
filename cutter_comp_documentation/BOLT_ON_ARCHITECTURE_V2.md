# Cutter Compensation v2.0: Bolt-On Architecture

**Design Date:** January 15, 2026  
**Status:** Design Phase  
**Goal:** Implement cutter compensation as a true "bolt-on" component that feeds process_move()

---

## Executive Summary

Version 2.0 refactors cutter compensation to work as a transparent filter in the G-code processing pipeline. Instead of creating a parallel execution stream with `ParsedMove` structures and `process_parsed_move()`, the compensator modifies G-code coordinates in place and feeds them to the existing `process_move()` function.

**Key Architectural Principle:** **"Gcode-In, Gcode-Out"**

---

## Problems with v1.0 Architecture

### Dual Execution Paths

```
v1.0 (Current - FLAWED):
G-code arrives
    ↓
Parse to ParsedMove struct
    ↓
Buffer ParsedMoves
    ↓
Apply compensation
    ↓
process_parsed_move() ← DUPLICATE coordinate transforms
    ↓
append_milestone()
```

**Problems:**
1. ❌ Duplicates 150+ lines of WCS transform logic
2. ❌ Manual `machine_position` tracking prone to bugs
3. ❌ Bypasses soft limits, feed overrides, and other Robot features
4. ❌ Two code paths to maintain and debug
5. ❌ Hard to diagnose issues (which path caused the problem?)

---

## v2.0 Architecture: Always-On Stream Processing

### Data Flow

```
v2.0 (Always-On Buffering):
G-code arrives → on_gcode_received()
    ↓
buffer_gcode() ← ALWAYS (moves and commands)
    ↓
Circular buffer (10 slots)
    ↓
get_compensated_gcode()
    ↓
Is compensation active?
   ↙         ↘
 YES          NO
  ↓            ↓
Apply offset   Pass through
calculations   unchanged
  ↓            ↓
Modify Gcode coordinates
  ↓
Return Gcode*
  ↓
process_move(Gcode*) ← UNIFIED PATH
  ↓
Standard WCS transforms
  ↓
append_line() / append_arc()
  ↓
Planner / Stepper
```

**Benefits:**
- ✅ Single execution path = easier debugging
- ✅ All Robot features work automatically
- ✅ No code duplication
- ✅ Compensation is truly "bolt-on"
- ✅ Always-on buffering provides lead-in context
- ✅ No conditional logic for when to buffer
- ✅ Stream-based processing = simpler architecture
- ✅ Minimal flush requirements

---

## Always-On Buffering Architecture

### Core Concept: Stream Processing

**Traditional Approach (v1.0):**
- Only buffer when compensation is active
- No knowledge of upcoming moves when G41/G42 encountered
- Cannot calculate proper lead-in offset

**Always-On Approach (v2.0):**
- Buffer ALL moves, regardless of compensation state
- When G41/G42 encountered, next moves already buffered
- Can calculate proper lead-in from buffered move direction
- Simpler logic - no conditional buffering

### How It Works

**Before G41/G42 (Compensation Off):**
```
G0 X10 Y10 → buffer → output immediately (uncompensated)
G1 X20 Y20 → buffer → output immediately (uncompensated)
G1 X30 Y30 → buffer → waiting in buffer
```
Buffer acts as 1-move delay, but compensation_type = NONE, so moves pass through unchanged.

**G41/G42 Activation:**
```
G1 X30 Y30 → already buffered!
G41 D6.0   → set compensation_type = LEFT, radius = 3.0
             → look at buffer[tail] to see next move is X30 Y30
             → calculate lead-in direction from current pos to X30 Y30
             → initialize compensated_position with offset
G1 X40 Y30 → buffer (will be compensated when output)
```
Lead-in is properly calculated because we can see the next move!

**During Compensation:**
```
G1 X40 Y30 → buffer
           → buffer_count = 3 (have lookahead)
           → apply_compensation() with corner detection
           → output compensated coordinates
G1 X40 Y40 → buffer (next move for corner lookahead)
```

**G40 Deactivation:**
```
G1 X50 Y50 → buffered, compensated
G40        → buffered as command
           → when G40 reaches front of buffer:
           → previous moves already output (compensated)
           → set compensation_type = NONE
           → subsequent moves pass through uncompensated
G0 Z5      → buffered, will output uncompensated
```
No explicit flush needed - stream naturally processes G40 in sequence!

### Buffer State Machine

```
STATE: Compensation OFF
  Buffer behavior: Pass through unchanged
  On G41/G42: Look ahead at buffered moves, calculate lead-in
  
STATE: Compensation ON
  Buffer behavior: Apply offset calculations
  Corner detection with lookahead
  
STATE: Compensation OFF (after G40)
  Buffer behavior: Pass through unchanged
  Already compensated moves were output before G40 processed
```

### Flush Requirements

**Explicit flush needed for:**
- Tool changes (T codes with M6) - physical synchronization required
- *(Optional)* End of program (M30) - but buffer naturally drains anyway

**No flush needed for:**
- ✅ G40 - stream processing handles naturally
- ✅ G41/G42 - look ahead at buffered moves
- ✅ M-codes (spindle, coolant) - buffer and pass through
- ✅ Non-move G-codes - buffer and pass through
- ✅ Program end - buffer drains naturally

### Lead-In Calculation Example

```cpp
// When G41/G42 encountered
void set_compensation(CompensationType type, float radius) {
    compensation_type = type;
    compensation_radius = radius;
    
    if(type != NONE && buffer_count > 0) {
        // Look at first buffered move
        BufferedGcode& first_move = buffer[buffer_tail];
        
        // Calculate direction from current position to first move endpoint
        float direction[2] = {
            first_move.endpoint[X_AXIS] - uncompensated_position[X_AXIS],
            first_move.endpoint[Y_AXIS] - uncompensated_position[Y_AXIS]
        };
        
        // Normalize
        float mag = sqrtf(direction[0]*direction[0] + direction[1]*direction[1]);
        if(mag > 0.0001f) {
            direction[0] /= mag;
            direction[1] /= mag;
        }
        
        // Calculate perpendicular offset for lead-in
        float normal[2];
        if(type == LEFT) {
            normal[0] = -direction[1];  // 90° CCW
            normal[1] = direction[0];
        } else {
            normal[0] = direction[1];   // 90° CW
            normal[1] = -direction[0];
        }
        
        // Apply lead-in offset
        compensated_position[X_AXIS] = uncompensated_position[X_AXIS] + normal[0] * radius;
        compensated_position[Y_AXIS] = uncompensated_position[Y_AXIS] + normal[1] * radius;
        compensated_position[Z_AXIS] = uncompensated_position[Z_AXIS];
    }
}
```

---

## Component Design

### 1. CompensationTypes.h (Unchanged)

```cpp
namespace Compensation {
    enum Side {
        NONE,
        LEFT,   // G41
        RIGHT   // G42
    };
}
```

### 2. CompensationPreprocessor Class

#### Key Design Decisions

**Buffer Storage:** Store Gcode* pointers, not ParsedMove structs
- Allows modification of original G-code objects
- Maintains compatibility with process_move()

**Memory Management:** Compensator owns cloned Gcode objects
- Clone incoming Gcode for buffering
- Delete after processing
- Clean up all on disable/destruct

**Coordinate Modification:** Rebuild command string with new coordinates
- Simple and uses existing G-code parser
- No need to modify Gcode class internals

#### Public API

```cpp
class CompensationPreprocessor {
public:
    /**
     * Buffer a G-code command (move or non-move)
     * 
     * ALWAYS call this for ALL commands - buffering is always active.
     * Clones the Gcode object internally. Original remains owned by caller.
     * 
     * @param gcode - G-code object to buffer (will be cloned)
     * @return true if buffered, false if buffer full
     */
    bool buffer_gcode(const Gcode* gcode);
    
    /**
     * Get next command from buffer (compensated if active)
     * 
     * Returns nullptr if need more lookahead (buffer_count < 3 when compensating).
     * Ownership transfers to caller - caller must delete returned Gcode*.
     * 
     * Always call after buffer_gcode() to maintain stream flow.
     * 
     * @return Gcode* if ready, nullptr if need more lookahead
     */
    Gcode* get_compensated_gcode();
    
    /**
     * Set compensation state
     * 
     * Called when G40/G41/G42 encountered.
     * Can look at buffered moves to calculate lead-in direction.
     * 
     * @param type - NONE, LEFT, or RIGHT
     * @param radius - Tool radius (NOT diameter)
     */
    void set_compensation(CompensationType type, float radius);
    
    /**
     * Check if compensation is active
     */
    bool is_active() const { return compensation_type != NONE; }
    
    /**
     * Get current buffer count (for debugging)
     */
    int get_buffer_count() const { return buffer_count; }
};
```

#### Internal Buffer Structure

```cpp
private:
    struct BufferedGcode {
        Gcode* gcode;              // Cloned Gcode (OWNED by buffer)
        float endpoint[3];         // Original endpoint (for geometry calc)
        float uncomp_start[3];     // Uncompensated start position (for arc centers)
        float ijk[3];              // Original I/J/K (for arcs)
        bool is_move;              // True if G0/G1/G2/G3, false for other commands
        bool has_ijk;              // True if arc (G2/G3)
        bool is_cw;                // True for G2, false for G3
        float direction[2];        // Cached XY unit vector (for lines)
    };
    
    static const int MAX_BUFFER_SIZE = 10;
    static const int LOOKAHEAD_DEPTH = 3;
    
    BufferedGcode move_buffer[MAX_BUFFER_SIZE];
    int buffer_head, buffer_tail, buffer_count;
```

#### Key Helper Functions

```cpp
/**
 * Clone a Gcode object and extract move parameters
 */
Gcode* clone_and_extract(
    const Gcode* original,
    MotionMode mode,
    float xyz[3],
    float ijk[3]
);

/**
 * Modify Gcode coordinates by rebuilding command string
 */
void modify_gcode_coordinates(
    Gcode* gcode,
    const float new_xyz[3],
    const float new_ijk[3]
);
```

### 3. Robot Integration

#### Minimal Changes to Robot::on_gcode_received()

**Current v1.0 code (lines 1220-1240):**
```cpp
if(comp_preprocessor && comp_preprocessor->is_active()) {
    ParsedMove parsed_move;
    parse_move_for_compensation(gcode, motion_mode, parsed_move);
    comp_preprocessor->buffer_move(parsed_move);
    while(comp_preprocessor->get_compensated_move(compensated_move)) {
        process_parsed_move(compensated_move);  // WRONG PATH
    }
}
```

**Proposed v2.0 code (Always-On Buffering):**
```cpp
if(motion_mode != NONE) {
    is_g123 = motion_mode != SEEK;
    
    // ALWAYS buffer moves (regardless of compensation state)
    comp_preprocessor->buffer_gcode(gcode);
    
    // Try to get next move from buffer
    Gcode* output_gcode = comp_preprocessor->get_compensated_gcode();
    
    if(output_gcode != nullptr) {
        // Feed to NORMAL path - unified execution
        process_move(output_gcode, motion_mode);
        
        // Clean up (we own the returned Gcode*)
        delete output_gcode;
    }
    // If nullptr, need more lookahead - command buffered for later
}

// Handle compensation state changes
if(gcode->has_g) {
    switch(gcode->g) {
        case 40: // G40 - Turn off compensation
            comp_preprocessor->set_compensation(NONE, 0.0f);
            break;
            
        case 41: // G41 - Left compensation
        case 42: // G42 - Right compensation
        {
            float radius = gcode->has_letter('D') ? gcode->get_value('D') / 2.0f : 0.0f;
            CompensationType type = (gcode->g == 41) ? LEFT : RIGHT;
            
            // Preprocessor can look at buffered moves for lead-in calculation
            comp_preprocessor->set_compensation(type, radius);
            break;
        }
    }
}
```

**Changes:**
- ✅ ~15 lines changed (simpler with always-on buffering)
- ✅ No conditional logic for when to buffer
- ✅ No new execution path
- ✅ Clean ownership semantics (delete after use)
- ✅ Lead-in calculation automatic from buffered moves

---

## Gcode Coordinate Modification Strategy

### Problem: Gcode is "Parsed and Cached"

The Gcode class parses the command string on construction and caches values. To modify coordinates, we need to:
1. Reconstruct the command string with new values
2. Free the old command string
3. Re-parse to update cached values

### Implementation

```cpp
void CompensationPreprocessor::modify_gcode_coordinates(
    Gcode* gcode,
    const float new_xyz[3],
    const float new_ijk[3]
) {
    // Build new command string
    std::string new_command = "G";
    if(gcode->has_g) {
        new_command += std::to_string(gcode->g);
    }
    
    // Add compensated XYZ
    if(!isnan(new_xyz[0])) {
        char buf[32];
        snprintf(buf, sizeof(buf), " X%.4f", new_xyz[0]);
        new_command += buf;
    }
    if(!isnan(new_xyz[1])) {
        char buf[32];
        snprintf(buf, sizeof(buf), " Y%.4f", new_xyz[1]);
        new_command += buf;
    }
    if(!isnan(new_xyz[2])) {
        char buf[32];
        snprintf(buf, sizeof(buf), " Z%.4f", new_xyz[2]);
        new_command += buf;
    }
    
    // Add compensated I/J/K for arcs
    if(!isnan(new_ijk[0])) {
        char buf[32];
        snprintf(buf, sizeof(buf), " I%.4f", new_ijk[0]);
        new_command += buf;
    }
    if(!isnan(new_ijk[1])) {
        char buf[32];
        snprintf(buf, sizeof(buf), " J%.4f", new_ijk[1]);
        new_command += buf;
    }
    
    // Preserve feed rate
    if(gcode->has_letter('F')) {
        char buf[32];
        snprintf(buf, sizeof(buf), " F%.1f", gcode->get_value('F'));
        new_command += buf;
    }
    
    // Replace command string
    free(gcode->command);
    gcode->command = strdup(new_command.c_str());
    
    // Re-parse to update cached values
    gcode->prepare_cached_values(false);
}
```

### Overhead Analysis

**String reconstruction cost:** ~50-100μs per move  
**Motion planning time:** ~5-10ms per move  
**Percentage overhead:** ~1-2% (negligible)

---

## Memory Comparison

| Component | v1.0 (ParsedMove) | v2.0 (Always-On) | Difference |
|-----------|-------------------|------------------|------------|
| Per-move storage | 80 bytes | 320 bytes | +240 bytes |
| 10-move buffer | 800 bytes | 3200 bytes | +2.4KB |
| Code size | +200 lines | -180 lines | **-380 lines net** |
| Execution paths | 2 paths | 1 path | **50% simpler** |
| Conditional logic | Complex | Minimal | **70% simpler** |
| Lead-in handling | Impossible | Automatic | **New capability** |

**Analysis:** The 2.4KB memory cost is trivial on LPC1768 (64KB RAM). The always-on architecture is even simpler than conditional buffering and enables proper lead-in calculation.

---

## Testing Strategy

### Phase 1: Basic Linear Moves

```gcode
G90 G54
G0 X0 Y0 Z5
G41 D6          ; Enable left compensation
G1 F150 X50 Y0  ; Simple line
G40             ; Cancel
```

**Verify:**
- ✅ Compensated Gcode fed to process_move()
- ✅ Coordinates show 3mm offset
- ✅ No crashes or errors

### Phase 2: Corners

```gcode
G90 G54
G0 X0 Y0 Z5
G41 D6
G1 F150
G1 X50 Y0       ; Line 1
G1 X50 Y50      ; Line 2 - outside corner
G1 X0 Y50       ; Line 3 - another corner
G1 X0 Y0        ; Line 4 - close
G40
```

**Verify:**
- ✅ Corner intersections calculated correctly
- ✅ Inside vs. outside corners handled
- ✅ No gaps or overlaps at corners

### Phase 3: Arcs

```gcode
G90 G54
G0 X0 Y0 Z5
G41 D6
G2 X50 Y0 I25 J0 F150  ; CW arc
G40
```

**Verify:**
- ✅ Arc endpoint offset correctly
- ✅ I/J modified appropriately
- ✅ Arc executes smoothly

---

## Implementation Checklist

### Step 1: Create Base Files on Fresh Branch
- [ ] Checkout `cutter-compensation-v2` from origin/master
- [ ] Add `CompensationTypes.h` (same as v1.0)
- [ ] Create `CompensationPreprocessor.h` with new API
- [ ] Create `CompensationPreprocessor.cpp` skeleton

### Step 2: Implement Gcode Manipulation
- [ ] Implement `clone_and_extract()`
- [ ] Implement `modify_gcode_coordinates()`
- [ ] Unit test: Verify round-trip works (parse → modify → parse)

### Step 3: Implement Buffer Management
- [ ] Circular buffer operations (push, pop, at)
- [ ] `buffer_gcode()` with cloning
- [ ] Lifetime management (destructor cleanup)

### Step 4: Port Compensation Math
- [ ] Copy `calculate_perpendicular_offset()` from extracted algorithms
- [ ] Copy `calculate_corner_intersection()`
- [ ] Copy `compensate_arc_endpoint()`
- [ ] Adapt to work with BufferedGcode structure

### Step 5: Implement Main API
- [ ] `get_compensated_gcode()` with corner detection
- [ ] `get_flushed_gcode()` for G40/M2
- [ ] `enable_compensation()` / `disable_compensation()`

### Step 6: Robot Integration
- [ ] Modify `on_gcode_received()` per design above
- [ ] Remove old `parse_move_for_compensation()`
- [ ] Remove old `process_parsed_move()`
- [ ] Update G40/G41/G42 handlers

### Step 7: Testing
- [ ] Build firmware
- [ ] Test Phase 1: Basic lines
- [ ] Test Phase 2: Corners
- [ ] Test Phase 3: Arcs
- [ ] Compare with v1.0 behavior

### Step 8: Cleanup
- [ ] Remove debug output
- [ ] Update comments
- [ ] Document changes in README

---

## Risk Mitigation

### Risk: Gcode Lifetime Bugs

**Mitigation:**
- Clear ownership rules (buffer owns clones, caller owns returned Gcode*)
- Destructor cleanup on disable_compensation()
- Unit tests for memory leaks

### Risk: Coordinate Modification Breaks Parsing

**Mitigation:**
- Test with all G-code variations (absolute, incremental, partial coordinates)
- Verify round-trip: original → modify → parse → matches expected

### Risk: Performance Regression

**Mitigation:**
- Benchmark string reconstruction overhead
- Profile critical path with real G-code
- Accept 1-2% overhead for 50% code complexity reduction

---

## Success Criteria

**v2.0 is successful if:**

1. ✅ Compensation works correctly (matches v1.0 behavior)
2. ✅ Single execution path (all moves through process_move())
3. ✅ Net code reduction (~150-200 lines removed)
4. ✅ Easier to debug (no dual-stream confusion)
5. ✅ Compatible with Robot features (soft limits, overrides)
6. ✅ Clean architecture for maintainers

---

## Future Enhancements (Post-v2.0)

- Optimize Gcode modification (avoid string reconstruction)
- Add lead-in/lead-out moves
- Implement singularity detection
- Add G-code validation warnings
- Tool table integration
- Advanced corner strategies (arc insertion)

---

**Document Version:** 1.0  
**Author:** GitHub Copilot (Claude Sonnet 4.5)  
**Date:** January 15, 2026
