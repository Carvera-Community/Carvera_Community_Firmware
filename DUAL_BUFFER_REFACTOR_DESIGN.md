# Dual Buffer Architecture Design
**Date**: March 10, 2026 (Updated March 24, 2026)  
**Purpose**: Refactor compensation preprocessor to use two dedicated ring buffers  
**Phase**: Bug fix for Phase 2 + preparation for Phase 3 corner calculations

---

## External References

**Industry Standards:**
- **LinuxCNC G41/G42 Documentation**: http://linuxcnc.org/docs/html/gcode/g-code.html#gcode:g41-g42
  - Standard behavior for cutter radius compensation
  - Entry/exit move requirements
  - Corner intersection calculations
  
- **NIST RS274NGC G-code Standard**: https://www.nist.gov/publications/nist-rs274ngc-interpreter-version-3
  - Official G-code specification (Section 3.5.16 - Cutter Radius Compensation)
  - Tool diameter offset behavior
  
- **Fanuc Programming Manual**: (Industry reference for CNC compensation behavior)
  - G40: Cancel cutter compensation
  - G41: Cutter compensation left
  - G42: Cutter compensation right
  - D-word: Tool diameter specification

---

## Architecture Overview

### Current Problem (March 10-20, 2026)
- Single buffer trying to track both uncompensated and compensated coordinates
- Confusion about when to update positions (buffering vs output time)
- `compute_and_output()` called repeatedly without buffer advancement (infinite loops)
- Two separate "clocks" driving the system (write path vs read path)
- `comp_count` growing unbounded (156+ with only 3 buffer slots)

### New Solution (REVISED March 24, 2026)
**Two synchronized ring buffers with SINGLE DRIVER:**
1. **Uncompensated Buffer** (`uncomp_ring`): Source of truth for programmed path
2. **Compensated Buffer** (`comp_ring`): Computed offset path for machine execution
3. **Single Clock**: `buffer_gcode()` drives write, compute, advance, and output handoff
4. **Push Handoff**: Robot consumes compensated output from `buffer_gcode()` return, not free polling
5. **Flush-Only Decrement**: `comp_count` does not decrement in normal G41/G42 streaming

---

## Buffer Structure

### Uncompensated Buffer
```cpp
struct UncompPoint {
    float x, y, z;
};

UncompPoint uncomp_ring[3];  // 3 slots for lookahead
int uncomp_head;              // Write position (next slot to fill)
int uncomp_tail;              // Read position (next slot to compute)
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
int comp_tail;                // Read position (next slot to serve)
int comp_count;               // Occupied/ready slots in comp_ring (fullness)
```

**Key Properties:**
- Each slot stores **ONE compensated point** + rebuilt Gcode
- `comp_head` is **locked** to `uncomp_tail` (synchronization point)
- Computes offset by looking back at last 2-3 slots in `uncomp_ring`
- Outputs actual machine commands

---

## Buffer Synchronization (REVISED March 24, 2026)

### Single Driver Model - Write Drives Everything

**CRITICAL PRINCIPLE**: The serial stream is the ONLY clock. When a G-code arrives, `buffer_gcode()` writes it, computes the offset, AND advances the buffers - all in one atomic operation.

**Old Problem (March 10-20)**:
```
WRITE CLOCK: buffer_gcode() → fills uncomp_ring → maybe calls compute_and_output()
READ CLOCK: get_compensated_gcode() → advances uncomp_tail and comp_tail
RESULT: Two drivers, unsynchronized, infinite loops in flush()
```

**New Solution (March 24)**:
```
SINGLE CLOCK: buffer_gcode() → write uncomp → compute_and_output() → advance state
             → serve comp tail (return Gcode*) on same tick
NORMAL STREAMING: comp_count tracks fullness and does NOT decrement
FLUSH (G40): explicit drain path decrements comp_count and uncomp_count to zero
RESULT: One driver, synchronized, deterministic drain behavior
```

### Synchronization Rules (REVISED)

1. **Write Phase**: `buffer_gcode(gcode)` stores point in `uncomp_ring[uncomp_head]`, advances head
2. **Compute Phase**: When `uncomp_count >= 3`, call `compute_and_output()` on the same clock tick
3. **Computation**:
   - Read from `uncomp_ring[uncomp_tail]` (and neighbors for direction)
   - Write to `comp_ring[comp_idx]` where `comp_idx = uncomp_tail` (synchronized)
    - **Advance uncomp_tail** and maintain comp fullness semantics
4. **Serve Phase (Normal G41/G42)**: `buffer_gcode()` returns one compensated `Gcode*` from comp tail
5. **Drain Phase (G40 flush)**: Decrement both rings intentionally until empty

### Second-Pass Contract (Authoritative, March 24, 2026)

This section overrides any contradictory text later in this document.

1. `comp_count` means "how full is `comp_ring`".
2. During normal streaming (G41/G42), `comp_count` is not decremented.
3. Advancement is clocked by `buffer_gcode()` only; no free-running pull loop.
4. `buffer_gcode()` always advances pipeline state when compensation is active.
5. During `flush()` (G40), decrement and drain both rings explicitly to zero.
6. `get_compensated_gcode()` is not part of normal streaming handoff; if retained, it is flush-only.
7. First-fill priming still computes two points once, but this must be guarded by persistent session state (not `comp_count == 0` alone).

### Buffer Advancement Pattern (Example)

**Initial State:**
```
uncomp_ring: [_, _, _]  head=0, tail=0, count=0
comp_ring:   [_, _, _]  count=0
```

**buffer_gcode(A):**
```
uncomp_ring: [A, _, _]  head=1, tail=0, count=1
comp_ring:   [_, _, _]  count=0
```
- Write A at slot 0, advance head to 1
- uncomp_count < 3, so no compute triggered

**buffer_gcode(B):**
```
uncomp_ring: [A, B, _]  head=2, tail=0, count=2
comp_ring:   [_, _, _]  count=0
```
- Write B at slot 1, advance head to 2
- uncomp_count < 3, so no compute triggered

**buffer_gcode(C):**
```
uncomp_ring: [A, B, C]  head=0, tail=0, count=3  ← Buffer full!
   ↓ Special case: comp_count == 0 (first fill)
   ↓ "Prime the pump" - compute TWO points back-to-back:
   
   ↓ FIRST compute_and_output():
   ↓   Computes A' using direction A→B (onset direction, comp_count==0 special case)
   ↓   Stores at comp_ring[0] (comp_idx = uncomp_tail = 0)
   ↓   Advances: uncomp_tail=1, comp_count=1 (uncomp_count stays 3)
   
   ↓ SECOND compute_and_output() (if uncomp_count >= 2):
   ↓   Computes B' using direction A→B→C (corner calculation, 3-point lookback)
   ↓   Stores at comp_ring[1] (comp_idx = uncomp_tail = 1)
   ↓   Advances: uncomp_tail=2, comp_count=2 (uncomp_count stays 3)
```
```
AFTER:
uncomp_ring: [A, B, C]  head=0, tail=2, count=3  ← Still full! Count stays at 3
comp_ring:   [A',B',_]  count=2                    ← A' and B' both computed
```
- Write C at slot 2, advance head to 0 (wraps)
- uncomp_count == 3 AND comp_count == 0, triggers **double compute**
- First: A' computed using onset direction (special case)
- Second: B' computed as corner using all 3 points (A→B→C)
- Tail advanced to 2 (read pointer moves), but **count stays 3** (all slots occupied)

**buffer_gcode(D):**
```
uncomp_ring: [D, B, C]  head=1, tail=2, count=3  ← Still full!
comp_ring:   [A',B',_]  count=2
```
- Write D at slot 0, advance head to 1 (overwrites old A)
- uncomp_count stays at 3 (buffer remains full)
- Check: uncomp_count (3) >= 3 AND comp_count (2) < 3? **YES**
- Triggers single compute:

```
   ↓ Normal single compute (comp_count > 0):
   ↓ Computes C' using direction B→C→D (corner calculation, 3-point lookback)
   ↓ Stores at comp_ring[2] (comp_idx = uncomp_tail = 2)
   ↓ Advances: uncomp_tail=0, comp_count=3 (uncomp_count stays 3)
```
```
AFTER:
uncomp_ring: [D, B, C]  head=1, tail=0, count=3  ← Still full!
comp_ring:   [A',B',C']  count=3                  ← Comp buffer full
```
- C' computed using B→C→D
- Tail advanced to 0 (read pointer moves)
- **uncomp_count stays 3** (buffer remains full with D, B, C data)

**buffer_gcode(E):**
```
(Assuming Robot.cpp has read A', so comp_count = 2)
uncomp_ring: [D, E, C]  head=2, tail=0, count=3  ← Still full!
   ↓ Normal single compute (comp_count > 0):
   ↓ Computes D' using direction C→D→E (corner calculation, 3-point lookback)
   ↓ Stores at comp_ring[0] (comp_idx = uncomp_tail = 0, overwrites A')
   ↓ Advances: uncomp_tail=1, comp_count=3 (uncomp_count stays 3)
```
```
AFTER:
uncomp_ring: [D, E, C]  head=2, tail=1, count=3  ← Still full!
comp_ring:   [D',B',C']  count=3                  ← Comp buffer full again
```
- Write E at slot 1, advance head to 2 (overwrites old B)
- uncomp_count stays at 3 (buffer remains full)
- Check: uncomp_count (3) >= 3 AND comp_count (2) < 3? YES
- Compute D' using C→D→E (normal single compute)
- Tail advanced to 1, **count stays 3**

**Key Insight: Why Double-Compute on First Fill?**

The double-compute pattern (computing both A' and B' when buffer first fills) serves two purposes:

1. **Start the output pipeline immediately**: Gets 2 compensated points ready so Robot.cpp can start moving
2. **Enable corner calculation for B'**: With all 3 points (A, B, C) available, B' can use full 3-point lookback for corner intersection logic (Phase 3)

After the initial double-compute, the system settles into a **single-compute cadence**: each new point triggers one compute, maintaining a 2-point lead (comp_count ≈ 2 during steady state).

**Important: uncomp_count Semantics**

`uncomp_count` represents **occupied slots** in the circular buffer, not "points waiting to be computed":
- Starts at 0 (empty)
- Grows 0→1→2→3 as points are buffered
- **Stays at 3** once buffer is full (during normal operation)
- Only decrements during `flush()` when draining the buffer

This is standard circular buffer behavior: the count tracks how full the buffer is, while `uncomp_tail` tracks the read position.

### Phase-Locked Relationship (SIMPLIFIED)

```cpp
// Storage index is ALWAYS synchronized with uncomp_tail
comp_idx = uncomp_tail;

// Buffer advancement in compute_and_output()
uncomp_tail = (uncomp_tail + 1) % BUFFER_SIZE;  // Read pointer advances
// uncomp_count stays constant (stays at 3 when buffer full)

comp_count++;  // Grows as we compute (bounded by BUFFER_SIZE)
```

### Key Benefits
- **No "last position" variable**: Phase offset provides lookback naturally
- **Full buffer utilization**: All 3 slots in both buffers are used
- **Simple synchronization**: Tails always at same position, head always tail+1
- **Clean initialization**: First point (A) outputs A' via onset direction on first-fill prime

---

## Calculation Logic (REVISED March 23, 2026)

### Phase 2: Simple Perpendicular Offset with First Point Special Case

**Key Insight**: Use `comp_count` to distinguish first point from subsequent points.

```cpp
void compute_and_output()
{
    // SAFETY: Don't exceed buffer capacity
    if (comp_count >= BUFFER_SIZE) {
        return;  // Comp buffer full, can't compute more
    }
    
    // Storage index synchronized with uncomp_tail
    int comp_idx = uncomp_tail;
    
    // SAFETY: Don't recompute if already have Gcode at this slot
    if (comp_ring[comp_idx].gcode != nullptr) {
        return;  // Already computed, skip
    }
    
    // First point vs subsequent point logic
    if (comp_count == 0) {
        // ================================================================
        // FIRST POINT: Compensate A using ONSET direction (A→B)
        // ================================================================
        
        int curr_idx = uncomp_tail;                  // Point A at tail
        int next_idx = (uncomp_tail + 1) % BUFFER_SIZE;  // Point B at tail+1
        
        const UncompPoint& curr = uncomp_ring[curr_idx];  // A
        const UncompPoint& next = uncomp_ring[next_idx];  // B
        
        // Calculate direction A→B
        float dx = next.x - curr.x;
        float dy = next.y - curr.y;
        float mag = sqrt(dx*dx + dy*dy);
        
        if (mag < 0.1f) {
            // Zero-length: no offset possible
            comp_ring[comp_idx].x = curr.x;
            comp_ring[comp_idx].y = curr.y;
            comp_ring[comp_idx].z = curr.z;
        } else {
            // Normalize and compute perpendicular
            dx /= mag;
            dy /= mag;
            
            float nx, ny;
            if (comp_type == LEFT) {
                nx = -dy;  // 90° CCW
                ny = dx;
            } else {
                nx = dy;   // 90° CW
                ny = -dx;
            }
            
            // Apply offset to point A (ONSET direction)
            comp_ring[comp_idx].x = curr.x + nx * comp_radius;
            comp_ring[comp_idx].y = curr.y + ny * comp_radius;
            comp_ring[comp_idx].z = curr.z;
        }
        
    } else {
        // ================================================================
        // SUBSEQUENT POINTS: Compensate using ENTRY direction (prev→curr)
        // ================================================================
        
        int prev_idx = uncomp_tail;                  // Previous point
        int curr_idx = (uncomp_tail + 1) % BUFFER_SIZE;  // Current point
        
        const UncompPoint& prev = uncomp_ring[prev_idx];
        const UncompPoint& curr = uncomp_ring[curr_idx];
        
        // Calculate direction prev→curr
        float dx = curr.x - prev.x;
        float dy = curr.y - prev.y;
        float mag = sqrt(dx*dx + dy*dy);
        
        if (mag < 0.1f) {
            // Zero-length: no offset possible
            comp_ring[comp_idx].x = prev.x;
            comp_ring[comp_idx].y = prev.y;
            comp_ring[comp_idx].z = prev.z;
        } else {
            // Normalize and compute perpendicular
            dx /= mag;
            dy /= mag;
            
            float nx, ny;
            if (comp_type == LEFT) {
                nx = -dy;  // 90° CCW
                ny = dx;
            } else {
                nx = dy;   // 90° CW
                ny = -dx;
            }
            
            // Apply offset to PREV point (entry direction)
            comp_ring[comp_idx].x = prev.x + nx * comp_radius;
            comp_ring[comp_idx].y = prev.y + ny * comp_radius;
            comp_ring[comp_idx].z = prev.z;
        }
    }
    
    // Build Gcode string
    char gcode_str[128];
    snprintf(gcode_str, sizeof(gcode_str), "G1 X%.3f Y%.3f Z%.3f",
             comp_ring[comp_idx].x, comp_ring[comp_idx].y, comp_ring[comp_idx].z);
    
    // Create and store Gcode object
    comp_ring[comp_idx].gcode = new Gcode(gcode_str, &StreamOutput::NullStream);
    
    // ADVANCE BUFFERS (Single driver model - this is the KEY change!)
    uncomp_tail = (uncomp_tail + 1) % BUFFER_SIZE;
    // Normal streaming: uncomp_count remains occupancy/fullness (stays at 3 once full)
    comp_count++;  // Fullness of comp_ring (no normal-stream decrement)
}
```

### Phase 3: Corner Intersections (Future)

When implementing Phase 3, the same `comp_count` check applies:
- `if (comp_count == 0)`: First point uses onset direction
- `else`: Subsequent points use corner intersection of two offset lines

**Lookback**: Use 3 consecutive uncomp points (tail-1, tail, tail+1) to calculate corner intersection

```cpp
// Phase 3 pseudocode (future):
Point A = uncomp_ring[(uncomp_tail - 1 + BUFFER_SIZE) % BUFFER_SIZE];  // Previous
Point B = uncomp_ring[uncomp_tail];                                     // Current (corner)
Point C = uncomp_ring[(uncomp_tail + 1) % BUFFER_SIZE];                // Next

// Calculate intersection of offset lines A'→B' and B'→C'
// Result becomes compensated point B' stored at comp[comp_idx]
```

**Initial State**: With phase-locking, the first compensated point is computed at position tail+1

**First Fill Sequence**:
```
Input A → uncomp[0], count=1
Input B → uncomp[1], count=2
Input C → uncomp[2], count=3 (buffer full)

State: uncomp = [A,B,C], tail=0, head=0
       comp_tail = 0, comp_head = 1
```

**First Compute/Output**:
- Compute: B' using direction A→B, store at comp[1]
- Output: comp[0] which is null/uninitialized
- **Result**: First point (A) effectively skipped, path starts at B'

**This is acceptable** because:
1. Only affects first point of program
2. Subsequent points (B, C, D, ...) all get proper offset
3. Phase 3 will handle this with proper lead-in calculation
4. Avoids needing "last position" tracking variable

### Zero-Length Moves

**Problem**: `G1 X50 Y50 Z10` (Z-only move, no XY change) has no direction for offset calculation.

**Solution with Phase-Locked Buffers**:
```cpp
// Computing comp[comp_head] where comp_head = (uncomp_tail+1)%3
int prev_idx = uncomp_tail;
int curr_idx = (uncomp_tail + 1) % 3;

float dx = uncomp_ring[curr_idx].x - uncomp_ring[prev_idx].x;
float dy = uncomp_ring[curr_idx].y - uncomp_ring[prev_idx].y;

if (sqrt(dx*dx + dy*dy) < 0.1f) {
    // Zero-length: output same coordinates (duplicate last position)
    comp_ring[comp_head].x = uncomp_ring[curr_idx].x;
    comp_ring[comp_head].y = uncomp_ring[curr_idx].y;
    comp_ring[comp_head].z = uncomp_ring[curr_idx].z;
}
```

**Behavior**: Zero-length moves output exact same Gcode command (duplicate), as confirmed by user.  
**Storage**: Always store in `uncomp_ring` even if zero-length.

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

## Algorithm Flow (REVISED March 24, 2026 - Clocked Push Model)

### Control Flow Diagram

```
Serial Stream → Parser → on_gcode_received() ← ⏰ THE ONLY CLOCK
                              │
                              ├─→ Comp OFF? → process_buffered_command()
                              ├─→ G40/G41/G42? → handle mode change
                              │
                              └─→ Comp ON, normal move:
                                  │
                                  └─→ buffer_gcode(gcode)
                                      ├─→ Write to uncomp_ring[uncomp_head]
                                      ├─→ uncomp_head++, uncomp_count++
                                      │
                                          └─→ IF uncomp_count >= 3:
                                          │
                                          └─→ compute_and_output()
                                              ├─→ Compute offset (if comp_count == 0: first point, else: subsequent)
                                              ├─→ Store in comp_ring[comp_idx]
                                              ├─→ uncomp_tail++ (normal streaming: uncomp_count stays occupied/full)
                                              └─→ comp_count tracks fullness (no decrement in normal stream)

                                          buffer_gcode() SERVES tail output immediately
                                          ├─→ Returns next compensated Gcode* for this tick
                                          ├─→ Robot consumes returned pointer (no free polling loop)
                                          └─→ During G40 flush only: explicit comp_count/uncomp_count decrement path
```

### Main Processing Functions

#### 1. `buffer_gcode()` - THE DRIVER (Write + Compute + Advance + Serve)
```cpp
Gcode* buffer_gcode(Gcode* gcode)
{
    // Check space
    if (uncomp_count >= BUFFER_SIZE) {
        return false;  // Full (shouldn't happen with proper advancement)
    }
    
    // Extract coordinates (modal if not specified)
    float x, y, z;
    if (uncomp_count == 0) {
        // First point must have explicit coords
        x = gcode->has_letter('X') ? gcode->get_value('X') : 0.0f;
        y = gcode->has_letter('Y') ? gcode->get_value('Y') : 0.0f;
        z = gcode->has_letter('Z') ? gcode->get_value('Z') : 0.0f;
    } else {
        // Modal coords from previous point
        int prev_idx = (uncomp_head - 1 + BUFFER_SIZE) % BUFFER_SIZE;
        x = gcode->has_letter('X') ? gcode->get_value('X') : uncomp_ring[prev_idx].x;
        y = gcode->has_letter('Y') ? gcode->get_value('Y') : uncomp_ring[prev_idx].y;
        z = gcode->has_letter('Z') ? gcode->get_value('Z') : uncomp_ring[prev_idx].z;
    }
    
    // WRITE to uncomp buffer
    uncomp_ring[uncomp_head].x = x;
    uncomp_ring[uncomp_head].y = y;
    uncomp_ring[uncomp_head].z = z;
    
    uncomp_head = (uncomp_head + 1) % BUFFER_SIZE;
    uncomp_count++;
    
    // COMPUTE if we have lookahead AND comp buffer has space
    // Special case: First fill (comp_count == 0) - "prime the pump" with 2 computes
    if (uncomp_count >= 3 && comp_count == 0) {
        // First fill - compute TWO points to start the pipeline
        compute_and_output();  // First point (A') using onset direction
        if (uncomp_count >= 2 && comp_count < BUFFER_SIZE) {
            compute_and_output();  // Second point (B') using corner calc
        }
    } else if (uncomp_count >= 3 && comp_count < BUFFER_SIZE) {
        // Normal case - single compute per buffer write
        compute_and_output();
    }
    
    // Serve one compensated output on this same clock tick.
    // Returns nullptr only when pipeline is still priming or no output is available.
    return served_output;
}
```

#### 2. `compute_and_output()` - Compute AND Advance Uncomp Buffer
```cpp
void compute_and_output()
{
    // SAFETY: Don't exceed comp buffer capacity
    if (comp_count >= BUFFER_SIZE) {
        return;
    }
    
    int comp_idx = uncomp_tail;  // Synchronized storage index
    
    // SAFETY: Don't recompute
    if (comp_ring[comp_idx].gcode != nullptr) {
        return;
    }
    
    // FIRST POINT vs SUBSEQUENT (see Calculation Logic section above for full code)
    if (comp_count == 0) {
        // Compute A' using A→B (onset direction)
        // ...
    } else {
        // Compute using prev→curr (entry direction)
        // ...
    }
    
    // Build and store Gcode
    char gcode_str[128];
    snprintf(gcode_str, sizeof(gcode_str), "G1 X%.3f Y%.3f Z%.3f", ...);
    comp_ring[comp_idx].gcode = new Gcode(gcode_str, &StreamOutput::NullStream);
    
    // ADVANCE BUFFERS ← This is the KEY to single-driver model!
    uncomp_tail = (uncomp_tail + 1) % BUFFER_SIZE;  // Move read pointer
    // Note: uncomp_count NOT decremented - stays at 3 when buffer full
    comp_count++;  // Increment computed count
}
```

#### 3. `get_compensated_gcode()` - OPTIONAL FLUSH ACCESSOR (Not Normal Streaming)
```cpp
Gcode* get_compensated_gcode()
{
    // Only used by explicit flush/drain paths if needed.
    // Not used by Robot.cpp as a free-running pull consumer.
    if (comp_count == 0) {
        return nullptr;  // Nothing computed yet
    }
    
    int comp_tail = uncomp_tail;  // Synchronized read index
    
    // Get the Gcode
    Gcode* result = comp_ring[comp_tail].gcode;
    
    // Clear the slot
    comp_ring[comp_tail].gcode = nullptr;
    
    // Decrement here only in explicit flush-drain behavior.
    comp_count--;
    
    return result;
}
```

#### 4. `flush()` - Empty Remaining Buffer (Clocked Drain)
```cpp
void flush()
{
    is_flushing = true;

    // Explicitly drain both rings to zero during G40.
    // This is the only path where counts are intentionally decremented.
    while (uncomp_count > 0 || comp_count > 0) {
        // compute/serve/decrement steps happen under flush policy
        step_flush_drain();
    }

    is_flushing = false;
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

## Implementation Backbone (March 24, 2026 - Second Pass)

This section is the implementation backbone for the second refactor pass.

### Required Invariants

1. `buffer_gcode()` is the only normal-stream clock.
2. `comp_count` means comp ring fullness and does not decrement during normal G41/G42 streaming.
3. Normal streaming always advances on `buffer_gcode()` ticks.
4. `flush()` (G40) is the only path that intentionally decrements and drains both rings.
5. First-fill double compute remains required, but must be guarded by persistent session state.

### Required API Direction

1. Update `buffer_gcode()` contract to return the compensated `Gcode*` for the current clock tick.
2. Remove Robot.cpp free polling loop in normal compensation flow.
3. Keep `get_compensated_gcode()` only as optional flush/drain accessor, or retire it once flush uses explicit drain step API.

### Code Work Items

1. `CompensationPreprocessor.h`
- [ ] Update `buffer_gcode` return type for push handoff.
- [ ] Add persistent prime/session state flag for first-fill logic.
- [ ] Keep explicit ring occupancy members for uncomp and comp buffers.

2. `CompensationPreprocessor.cpp`
- [ ] In `buffer_gcode()`: write, compute, serve one output, and advance every active tick.
- [ ] In `compute_and_output()`: keep onset-vs-entry logic; advance uncomp tail; keep normal-stream occupancy semantics.
- [ ] Prevent re-entry of first-fill logic after startup by using persistent session flag (not `comp_count == 0` alone).
- [ ] In normal stream: do not decrement `comp_count`; maintain cap at `BUFFER_SIZE`.
- [ ] In `flush()`: explicitly decrement `comp_count` and uncomp occupancy to zero with guaranteed forward progress.
- [ ] Add/confirm final drain behavior for trailing uncomp/comp points (reverse/special flush compute if needed).

3. `Robot.cpp`
- [ ] Replace normal pull loop with one-per-tick consumption from `buffer_gcode()` return.
- [ ] Keep flush integration deterministic and bounded.

### Validation Checklist

- [ ] Build succeeds after API contract update.
- [ ] No diagonal artifacts in square D-word test.
- [ ] D10 behavior is 5 mm radius in logs and motion.
- [ ] Normal streaming shows no `comp_count` decrement events.
- [ ] G40 flush drains both rings fully with deterministic termination.
- [ ] G41→G40→G42 transitions remain stable.

### Active Decisions (Locked)

1. Push handoff contract: `buffer_gcode()` returns compensated output.
2. Fullness semantics: `comp_count` is fullness, not pull queue depth.
3. Decrement policy: decrement only during explicit flush drain.
4. Clock policy: always advance on `buffer_gcode()` during active compensation.

---

## End of Document

**Document Version**: March 24, 2026 (Clocked Push Second Pass)
**Status**: Backbone updated for second refactor pass
**Next Action**: Implement the second-pass code changes using this section as authoritative guidance
