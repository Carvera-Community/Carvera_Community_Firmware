# Cutter Compensation Implementation Progress Report

## Overview
This document tracks the implementation of G41/G42 cutter compensation (tool radius compensation) in the Carvera Community Firmware, which is based on Smoothieware.

## Project Goal
Implement full G41 (left compensation) and G42 (right compensation) support to automatically offset tool paths by the tool radius, allowing accurate machining when the programmed path represents the final part geometry rather than the tool center path.

## Current State (April 2026)

This section is the authoritative status of the firmware as currently tested.
Historical architecture notes in the rest of this document remain useful context,
but this section should be used for current behavior and expectations.

### Implemented and Working

- Cutter compensation is integrated in the active motion path on branch `cutter-compensation-v2`.
- G41/G42 compensation works for G17 XY motion with line and arc sequences.
- Arc endpoint projection now uses the geometric offset radius (`uncomp_radius +/- tool_radius`) instead of deriving radius from compensated start-point drift.
- Terminal arc handling uses incoming tangent direction at segment end, preventing synthetic outgoing-direction artifacts on final compensated arc output.
- ARC validation suite (`ARC_01` through `ARC_06`) has passed in machine testing with no compensation hard faults and `COMP_LB miss=0.0%`.
- Representative Fusion-style finishing program (`cutter comp v6 proper lead ins finishing pass.cnc`) has passed with full compensated throughput (`in=68`, `gen=68`, `srv=68`, `miss=0.0%`).

### Operational Requirements

- Programmed lead-in and lead-out moves are required for reliable cutter compensation in production jobs.
- In particular, avoid enabling G41/G42 and making the very first compensated move an arc without a programmed approach move.
- Fusion 360 style posted lead-in/out geometry is the expected workflow and is aligned with current firmware behavior.

### Known Constraints

- R-format arcs with active compensation are not supported; use I/J arc format.
- Compensation logic is XY-focused (G17). Mixed-plane sections (for example G19 segments while compensation is active) should be treated as a higher-risk area and verified carefully on parts.
- Intermittent `list index out of range` lines observed near `G28` in Kivy logs appear host-side and are not currently attributed to compensation pipeline failure.

### Recommended Validation Pattern

- Validate with both canonical ARC tests and representative Fusion programs.
- For each run, verify:
   - no firmware hard-fault markers (ALARM/hard error/abort)
   - stable compensation activation/deactivation (G41/G42/G40)
   - healthy load-balance metrics (`COMP_LB miss=0.0%`, `prod_vs_pull` near 100%)

---

## Why Preprocessing is Required

### The Fundamental Problem
Cutter compensation cannot be applied move-by-move in real-time as G-code commands arrive because proper compensation requires knowledge of **future moves** to handle corners correctly. Consider this scenario:

```gcode
G41 D6          ; Enable left compensation, 6mm tool (3mm radius)
G1 X50 Y0       ; Move 1: Horizontal line
G1 X50 Y50      ; Move 2: Vertical line (90° corner)
```

**The Challenge:**
- When processing Move 1, we need to know about Move 2 to calculate the correct corner intersection
- A simple perpendicular offset of Move 1 would create a gap or overlap at the corner
- The compensation path must smoothly transition around corners without stopping

### Why Simple Per-Move Offsetting Fails

**Attempt 1: Offset Each Move Independently**
```
Original path:     →→→→→
                        ↑
                        ↑
                        
Simple offset:     →→→→→   ← Gap at corner!
                      ↑
                      ↑
```
Result: Gaps at outside corners, overlaps at inside corners, tool gouges the part.

**Attempt 2: Offset Only When Move Arrives**
- By the time we receive Move 2, the robot has already executed Move 1
- Cannot retroactively fix the corner
- Machine would need to stop, back up, and recalculate

### The Preprocessing Solution

**Preprocessor Architecture:**
1. **Buffer Incoming Moves** - Store recent moves in a lookahead buffer
2. **Analyze Move Relationships** - Determine corner angles, inside vs outside corners
3. **Calculate Intersection Points** - Find where offset paths intersect at corners
4. **Modify Target Coordinates** - Update move endpoints before they reach the planner
5. **Pass to Motion System** - Send corrected moves to existing planner/conveyor system

**Why This Works:**
- ✅ Has visibility into future moves before execution
- ✅ Can calculate proper corner intersections
- ✅ Integrates before the motion planner, so existing system handles execution
- ✅ Can insert additional moves if needed (lead-in/lead-out)
- ✅ Maintains smooth motion without stops

### Corner Types and Required Strategies

**Outside Corners (Convex):**
```
      Original        Compensated
        |                 |
    ----+             ----+----
                          |
                          |
```
- Offset paths intersect beyond the corner
- Need to calculate intersection point
- Tool travels farther than programmed path

**Inside Corners (Concave):**
```
    Original        Compensated
    ----+----       ----|
        |               +----
        |               |
```
- Offset paths intersect before the corner
- May need to limit intersection to avoid gouging
- In tight corners, may need to insert arc moves
- Tool travels shorter than programmed path

**Collinear Moves:**
```
    Original: ----→----→----→
    
    Compensated: ----→----→----→  (same offset maintained)
```
- Moves in same direction need consistent offset
- No corner calculation needed
- Must detect to avoid unnecessary computation

### Lookahead Requirements

**Minimum Lookahead: 2 moves**
- Current move + next move = enough to detect corners
- Can calculate basic intersection points
- Sufficient for simple rectangular paths

**Optimal Lookahead: 3+ moves**
- Previous + current + next = better context
- Can detect move sequences and optimize
- Can plan for sharp corners in advance
- Enables more sophisticated strategies

**Current Implementation:**
```cpp
static const int LOOKAHEAD_SIZE = 3;
std::vector<Move> move_buffer;
```

### Integration with Existing Motion System

The preprocessor sits **between G-code reception and motion planning:**

```
G-code Input
    ↓
[G40/G41/G42 Detection] ← Robot::on_gcode_received()
    ↓
[CompensationPreprocessor] ← Buffers, analyzes, offsets
    ↓
Modified Target Coordinates
    ↓
[Planner] ← Existing motion planning (unchanged)
    ↓
[Conveyor] ← Move queue (unchanged)
    ↓
[Stepper Motors] ← Execution (unchanged)
```

**Key Integration Points:**
1. **Robot::on_gcode_received()** - Detects G41/G42/G40, enables/disables preprocessing
2. **Before append_milestone()** - Calls preprocessor to modify target coordinates
3. **Coordinate Space** - Preprocessor works in workpiece coordinates (after G54/G92 applied)
4. **Segmentation** - Preprocessor must handle line segments created by `mm_per_line_segment`

### Alternative Approaches Considered and Rejected

**1. Post-Processing (CAM-Side)**
- ❌ Requires modifying every CAM post-processor
- ❌ User must manually specify tool radius in CAM
- ❌ No flexibility for tool changes
- ❌ Errors in CAM offset cannot be corrected at machine

**2. Real-Time Offsetting Without Lookahead**
- ❌ Cannot handle corners correctly
- ❌ Creates gaps and overlaps
- ❌ Would require machine stops at every corner
- ❌ Violates CNC control standards

**3. Buffering in the Planner**
- ❌ Planner deals with step timing, not geometry
- ❌ Too late to modify coordinates
- ❌ Would require major rewrite of motion system
- ❌ Affects real-time performance

**4. Hardware-Level Compensation**
- ❌ LPC1768 microcontroller has limited processing power
- ❌ Real-time corner calculations too expensive
- ❌ Better to do geometry before motion planning
- ❌ Preprocessing allows using existing optimized planner

### Why Preprocessing is the Industry Standard

Major CNC controls (Fanuc, Haas, Siemens, etc.) all use preprocessing for cutter compensation because:

1. **Geometric Correctness** - Only way to guarantee proper corner handling
2. **Performance** - Calculate once during preprocessing vs. repeatedly in real-time
3. **Separation of Concerns** - Geometry calculation separate from motion execution
4. **Flexibility** - Can implement sophisticated strategies (arcs at corners, spiral entry, etc.)
5. **Standards Compliance** - Matches expected G-code behavior per RS274NGC standard

### Preprocessing vs. Real-Time Processing Trade-offs

**Preprocessing Advantages:**
- ✅ Perfect corner handling
- ✅ Can look ahead multiple moves
- ✅ Can insert additional moves (lead-in/out)
- ✅ Predictable, debuggable behavior
- ✅ No real-time performance impact

**Preprocessing Challenges:**
- ⚠️ Requires move buffering
- ⚠️ Adds slight latency (usually imperceptible)
- ⚠️ Must handle move segmentation
- ⚠️ More complex architecture

**Conclusion:** The benefits far outweigh the challenges, which is why this approach was chosen after extensive analysis.

---

## v2.0 Always-On Buffering Architecture

### Evolution from Conditional to Always-On Buffering

**v1.0 Approach (Conditional Buffering):**
- Only buffer moves when compensation is active (G41/G42)
- Problem: No lookahead when G41/G42 encountered
- Cannot calculate proper lead-in offset direction
- Results in discontinuities at arc starts

**v2.0 Approach (Always-On Buffering):**
- Buffer ALL moves and commands, regardless of compensation state
- When G41/G42 encountered, next moves are already buffered
- Can look ahead to calculate proper lead-in direction
- Stream-based processing - commands flow sequentially through buffer

### Lead-In Problem Solved

**The Lead-In Challenge:**
```gcode
G0 X-7.071 Y7.071    ; Position before compensation
G42 D3.175           ; Activate compensation - but which direction to offset?
G2 X7.071 Y-7.071 I7.071 J-7.071  ; Arc is first move!
```

**Without lookahead:** Can't calculate initial offset - don't know arc direction yet.

**With always-on buffering:**
1. G0 move buffered and executed (uncompensated)
2. G42 buffered (will activate compensation when it reaches front)
3. G2 arc **already buffered** when G42 activates!
4. G42 looks at buffered arc, calculates entry direction
5. Initializes `compensated_position` with proper offset
6. Arc executes with correct start position

### Stream Processing Flow

**All commands buffer, all commands flow:**
```
[Input] → [Buffer] → [Apply Compensation if Active] → [Output]
```

- Non-move commands (M3, M8, etc.) buffer and pass through unchanged
- G40/G41/G42 buffer and change compensation state when output
- Moves buffer and get compensated (or not) based on current state
- Natural sequential processing - no special flush logic needed

### Minimal Flush Requirements

**Flush needed only for:**
- Tool changes (T codes with M6) - physical synchronization required
- Buffer naturally drains at program end

**No flush needed for:**
- G40 - processes naturally in stream, subsequent moves uncompensated
- G41/G42 - look ahead at buffered moves for lead-in
- M-codes - buffer and pass through
- Program end - buffer drains naturally to empty

### Benefits of Always-On Architecture

✅ **Proper lead-in** - Can see next move when compensation activates  
✅ **Simpler logic** - No conditional "should I buffer this?" checks  
✅ **Natural G40** - No explicit flush, sequential stream processing  
✅ **Arc-to-arc transitions** - Always have context from buffered moves  
✅ **Easier debugging** - Single consistent code path  
✅ **Minimal memory cost** - 2.4KB buffer on 64KB RAM system (trivial)  

---

## Architecture Changes

### New Files Created

#### 1. `src/modules/robot/CompensationTypes.h`
**Purpose:** Defines shared enums and types for the compensation system

**Contents:**
- `Compensation::Side` enum with values:
  - `NONE` = 0
  - `LEFT` = 1
  - `RIGHT` = 2
- Provides namespace to avoid circular dependencies between Robot and CompensationPreprocessor

**Status:** ✅ Complete and stable

---

#### 2. `src/modules/robot/CompensationPreprocessor.h`
**Purpose:** Header file declaring the CompensationPreprocessor class

**Key Components:**
- **Move Struct:**
  ```cpp
  struct Move {
      float start[2];      // XY start position
      float end[2];        // XY end position
      bool is_arc;
      bool clockwise;      // For arcs
      float center[2];     // For arcs (I,J)
      float radius;        // For arcs
      int line_number;
      float length;        // Cached move length
      float direction[2];  // Cached unit vector
  };
  ```

- **Public Methods:**
  - `enable_compensation(CompSide side, float diameter)` - Activates compensation
  - `disable_compensation()` - Deactivates compensation
  - `preprocess_move(Gcode* gcode, float* target, float* position)` - Main processing entry point
  - `preprocess_arc_offsets(float offset[2], bool clockwise)` - Arc handling
  - `is_active()` - Returns compensation state

- **Private Methods:**
  - `calculate_line_offset(const Move& move, float* output)` - Calculates perpendicular offset
  - `calculate_intersection(const Move& m1, const Move& m2, float* output)` - Corner handling
  - `buffer_move(const Move& move)` - Stores moves for lookahead
  - `flush_moves()` - Clears move buffer

**Status:** ✅ Header complete, implementation in progress

---

#### 3. `src/modules/robot/CompensationPreprocessor.cpp`
**Purpose:** Implements the CompensationPreprocessor class

**Current Implementation:**

**`enable_compensation(CompSide side, float diameter)`**
- Validates input side (NONE, LEFT, or RIGHT)
- Stores compensation radius (diameter / 2)
- Clears move buffer
- Provides debug output

**`preprocess_move(Gcode* gcode, float* target, float* position)`**
- Main entry point for move processing
- Validates compensation state
- Filters out tiny moves (< 0.1mm) to handle segmented moves
- Creates Move struct with cached length and direction
- Buffers the move for lookahead
- Calculates offset using `calculate_line_offset()`
- Updates target coordinates with compensated position
- Returns true if move was processed, false if passed through

**`calculate_line_offset(const Move& move, float* output)`**
- Uses cached direction vector from Move struct
- Calculates perpendicular normal vector:
  - LEFT (G41): Rotate 90° CCW (nx = -uy, ny = ux)
  - RIGHT (G42): Rotate 90° CW (nx = uy, ny = -ux)
- Applies offset: `output = endpoint + normal * radius`
- Provides detailed debug output

**`preprocess_arc_offsets(float offset[2], bool clockwise)`**
- Adjusts arc radius by modifying I,J offsets
- Logic:
  - G41 + G2 (CW) or G42 + G3 (CCW): add radius
  - G41 + G3 (CCW) or G42 + G2 (CW): subtract radius
- Scales I,J proportionally to change radius

**Status:** 🟡 Basic functionality implemented, debugging zigzag issue

---

#### 4. `src/modules/robot/corner_handling.cpp`
**Purpose:** Implements advanced corner compensation algorithms

**Functions:**
- Corner intersection calculations
- Inside vs outside corner detection
- Lead-in/lead-out move generation

**Status:** 📦 Compiled but not yet integrated into main flow

---

### Modified Files

#### 1. `src/modules/robot/Robot.h`
**Changes Made:**
- Added `#include "CompensationTypes.h"`
- Added forward declaration: `class CompensationPreprocessor;`
- Added member variables:
  ```cpp
  CompensationPreprocessor* comp_preprocessor;
  bool compensation_active;
  Compensation::Side comp_side;
  float compensation_radius;
  bool has_next_move;
  ```
- Added method declarations:
  ```cpp
  void set_compensation(Compensation::Side side, float radius);
  Compensation::Side get_compensation_side() const;
  ```

**Status:** ✅ Complete

---

#### 2. `src/modules/robot/Robot.cpp`
**Changes Made:**

**Constructor:**
```cpp
Robot::Robot() : 
    comp_preprocessor(new CompensationPreprocessor()),
    compensation_active(false),
    comp_side(Compensation::NONE),
    compensation_radius(0),
    has_next_move(false)
```

**Destructor:**
```cpp
Robot::~Robot() {
    delete comp_preprocessor;
}
```

**`set_compensation()` Method:**
- Validates side parameter
- Updates compensation state
- Calls `comp_preprocessor->enable_compensation()` or `disable_compensation()`
- Provides debug output

**`on_gcode_received()` G40/G41/G42 Handling:**
- **G40:** Disables compensation
  ```cpp
  set_compensation(Compensation::NONE, 0);
  ```
- **G41/G42:** Reads D word for tool diameter, enables compensation
  ```cpp
  float diameter = gcode->has_letter('D') ? gcode->get_value('D') : 6.0f;
  set_compensation(side, diameter);
  ```

**Move Processing Integration:**
- Added compensation check before append_milestone:
  ```cpp
  if (compensation_active && comp_preprocessor) {
      bool consumed = comp_preprocessor->preprocess_move(gcode, target, machine_position);
      if (consumed) {
          // Move was modified by compensation
      }
  }
  ```

**Status:** ✅ Basic integration complete

---

#### 3. `src/modules/tools/endstops/Endstops.cpp`
**Changes Made:**
- Added `#include "CompensationTypes.h"` for enum compatibility
- Fixed compilation issues with compensation type references

**Status:** ✅ Complete

---

## Test Files

### `tests/TEST_basic_cutter_comp/TEST_phase1_G4x_D_word.cnc`
**Purpose:** Basic validation test for cutter compensation

**Test Cases:**
1. **Test 1:** Simple line with G41 (left compensation)
   - Move from X0 Y0 to X50 Y0 with 6mm tool
   - Expected: 3mm offset to the left (Y+3)

2. **Test 2:** Simple line with G42 (right compensation)
   - Move from X0 Y0 to X50 Y0 with 6mm tool
   - Expected: 3mm offset to the right (Y-3)

3. **Test 3:** Outside corner with G41
   - X50 Y0, then X50 Y50
   - Expected: Maintain 3mm offset around 90° corner

4. **Test 4:** Inside corner with G42
   - X50 Y0, then X50 Y-50
   - Expected: Maintain 3mm offset around inside corner

5. **Test 5:** CW arc with G41
   - G2 X50 Y0 I25 J0
   - Expected: Increase arc radius by 3mm

6. **Test 6:** CCW arc with G42
   - G3 X50 Y0 I25 J0
   - Expected: Decrease arc radius by 3mm

**Status:** 📝 Test file created, currently debugging execution issues

---

## Issues Encountered and Solutions

### Issue 1: Circular Dependencies
**Problem:** Robot.h and CompensationPreprocessor.h needed each other

**Solution:**
- Created `CompensationTypes.h` with shared enum definitions
- Used forward declarations in Robot.h
- Included full headers only in .cpp files

**Status:** ✅ Resolved

---

### Issue 2: Enum Comparison Warnings
**Problem:** Compiler warnings about comparing enum with `<` and `>` operators
```
warning: comparison is always false due to limited range of data type
```

**Solution:**
Changed from:
```cpp
if (side < Compensation::NONE || side > Compensation::RIGHT)
```
To:
```cpp
if (side != Compensation::NONE && side != Compensation::LEFT && side != Compensation::RIGHT)
```

**Status:** ✅ Resolved

---

### Issue 3: Missing Closing Brace
**Problem:** Extra closing brace caused compilation error

**Solution:**
- Removed duplicate closing brace after `calculate_line_offset()` function

**Status:** ✅ Resolved

---

### Issue 4: Zigzag Motion on Single Line
**Problem:** When executing a single G1 move with compensation enabled, the machine creates a zigzag pattern, alternating between compensated and uncompensated positions.

**Debug Output Analysis:**
```
DBG:CompPrep: Processing move [-179.000,-124.000] -> [-129.000,-124.000]
DBG:CutterComp: Move #78 from [-179.000,-124.000] to [-174.000,-124.000]
DBG:CutterComp: Applied offset 3.000 -> [-174.000,-121.000]  // Offset applied
DBG:CutterComp: Move modified from [-169.000,-124.000] to [-169.000,-124.000]  // No offset
DBG:CutterComp: Applied offset 3.000 -> [-164.000,-121.000]  // Offset applied
DBG:CutterComp: Move modified from [-159.000,-124.000] to [-159.000,-124.000]  // No offset
```

**Root Causes Identified:**
1. **Line Segmentation:** The firmware breaks long moves into segments (controlled by `mm_per_line_segment` config, default 5mm)
2. **Alternating Processing:** Every other segment was getting compensated
3. **Coordinate Space Confusion:** Positions in machine coordinates vs. workpiece coordinates
4. **Inconsistent Offset Direction:** Short segments weren't maintaining consistent offset direction

**Solutions Implemented:**

1. **Added Move Direction Caching:**
   - Enhanced Move struct with `length` and `direction[2]` fields
   - Pre-calculate and store unit vectors when move is created
   - Maintains consistent direction across segments

2. **Short Move Handling:**
   ```cpp
   if (move_length < 0.1f && !move_buffer.empty()) {
       const Move& prev = move_buffer.back();
       target[0] = position[0] + dx + prev.direction[1] * comp_radius;
       target[1] = position[1] + dy - prev.direction[0] * comp_radius;
       return true;
   }
   ```
   - Short moves inherit direction from previous move
   - Prevents oscillation on segmented moves

3. **Simplified Offset Calculation:**
   ```cpp
   void CompensationPreprocessor::calculate_line_offset(const Move& move, float* output) {
       float ux = move.direction[0];
       float uy = move.direction[1];
       
       float nx, ny;
       if (comp_side == Compensation::LEFT) {
           nx = -uy;  // Rotate 90° CCW
           ny = ux;
       } else {
           nx = uy;   // Rotate 90° CW
           ny = -ux;
       }
       
       output[0] = move.end[0] + nx * comp_radius;
       output[1] = move.end[1] + ny * comp_radius;
   }
   ```
   - Uses cached direction vectors
   - Consistent normal vector calculation
   - Direct application of offset

4. **Removed Complex Buffering:**
   - Simplified move processing to handle each move immediately
   - Kept buffer for future corner handling but process inline for straight lines

**Status:** 🟡 Solutions implemented, awaiting recompile and test

---

### Issue 5: Coordinate Space Mismatch
**Problem:** Debug output shows positions like `[-179.000,-124.000]` when commanded position was `X0 Y0`

**Analysis:**
- This indicates machine coordinate space (including G54 work offset and G92 offset)
- Compensation needs to work in workpiece coordinates, not machine coordinates
- The start position for the test may be set in machine coordinates

**Potential Solutions:**
1. Ensure compensation calculations use workpiece coordinates
2. Transform coordinates appropriately before and after compensation
3. Verify that position[] and target[] parameters are in correct coordinate space

**Status:** 🔍 Under investigation

---

### Issue 6: Buffer Immediately Draining (Phase 1)
**Problem:** "buffering infrastructure is not buffering anything. It only appears to accept and then immediately release the code"

**Symptom:** Every command entering buffer immediately exited, no holding for lookahead

**Root Cause:** Original Phase 1 implementation had no lookahead requirement check in `get_compensated_gcode()`. Function always returned command if buffer had anything.

**Diagnosis:**
- Debug output showed every `>>BUFFER:` immediately followed by `>>OUTPUT:`
- No `>>PHASE1_BUFFERING:` messages indicating lookahead wait
- Buffer was functioning as pass-through instead of accumulator

**Solution Implemented:**
Added lookahead requirement check at CompensationPreprocessor.cpp lines 215-220:
```cpp
int required_lookahead = 3;
if (buffer_count < required_lookahead && !is_flushing) {
    THEKERNEL->streams->printf(">>PHASE1_BUFFERING: Holding for lookahead (count=%d, need=%d)\n", 
        buffer_count, required_lookahead);
    return nullptr;
}
```

**Result:**
- Buffer now holds first 2 commands
- 3rd command triggers output of 1st command
- Maintains 2-3 commands buffered in steady state
- Simulates real compensation lookahead needs

**Status:** ✅ Resolved in firmware_v2.2_PHASE1_SIMPLE.bin

---

### Issue 7: Hard Limit -X Errors from Corrupted Coordinates (Phase 1)
**Problem:** `ALARM: Hard limit -X` after entering 2-3 commands, log showed `X-2.0000 Y-2.0000` when original was `X0 Y0`

**Symptom:** Commands showing coordinates like X-2.0 Y-2.0 when original was X0 Y0

**Root Cause:** Buffer slots contained garbage/stale data from previous operations, never cleared at startup or between operations

**Diagnostic Evidence:**
```
>>BUFFER: G0 X0 Y0 (count=2, comp=OFF)  ← First command, but count ALREADY 2!
>>SLOT_DATA: endpoint=[19283832040946708267123251085312.000, ...] ← Garbage numbers
>>SLOT_GCODE: '' ← Empty or corrupted command
```

**Initial Fix Attempt:** Added `clear()` call in `set_compensation()` - partially helped but count still started at 2

**Real Fix:** Added pass-through mode when compensation OFF (Robot.cpp lines 615-627):
```cpp
if (!compensation_preprocessor->is_active()) {
    gcode->stream->printf(">>PASSTHROUGH: %s (comp=OFF, no buffering)\n", gcode->get_command());
    process_buffered_command(gcode);
    return;
}
```

**Why This Works:**
- When compensation OFF, commands never enter buffer
- Can't encounter stale data if not using buffer
- Simpler than trying to clear/reset buffer constantly
- Only buffer when actually needed (compensation ON)

**Additional Safety:** Added null pointer validation at CompensationPreprocessor.cpp lines 223-230:
```cpp
if (slot.gcode == nullptr) {
    THEKERNEL->streams->printf(">>ERROR: Buffer slot %d has NULL gcode pointer! Clearing buffer.\n", buffer_tail);
    clear();
    return nullptr;
}
```

**Result:**
- No more hard limit errors
- Coordinates always correct
- System auto-recovers if corruption detected

**Status:** ✅ Resolved in firmware_v2.2_PHASE1_SIMPLE.bin

---

### Issue 8: G40 Not Flushing All Commands (Phase 1 - THE CRITICAL FIX)
**Problem:** "when the buffer gets flushed via G40 not all of the commands will clear"

**Symptom:** Some commands remained in buffer after G40 executed

**User Insight:** "I dont think that the flush was or is deleting any moves. I think that we need to be paying attention to the pointers and the circular buffer"

**Initial Investigation:**
- Suspected circular buffer pointer corruption
- Suspected flush logic not properly decrementing count
- Added extensive diagnostic logging for buffer operations

**Root Cause Discovery:**
G40 itself was being BUFFERED as a regular command! When compensation was ON, G40 went into buffer, sat waiting for lookahead requirement instead of executing its flush logic.

**Sequence With Bug:**
```
G41 D10 → bypasses buffer, activates compensation
G1 X10 Y0 → buffered (count=1)
G1 X10 Y10 → buffered (count=2)
G40 → BUFFERED (count=3)! ← WRONG! Should execute immediately
→ First command outputs, but G40 is stuck in buffer waiting its turn
```

**The Problem:**
Control commands that manage the buffer cannot themselves be subject to buffering - creates circular dependency.

**Solution Implemented:**
Added bypass check in `Robot::on_gcode_received()` BEFORE buffering logic (Robot.cpp lines 628-637):
```cpp
// CRITICAL: G40/G41/G42 must NEVER be buffered - they control the buffering system itself!
// G40 needs to flush the buffer, so it must execute immediately
// G41/G42 need to clear/reset the buffer, so they must execute immediately
if (gcode->has_g && (gcode->g == 40 || gcode->g == 41 || gcode->g == 42)) {
    gcode->stream->printf(">>BYPASS_BUFFER: G%d %s (compensation control command)\n", 
        gcode->g, gcode->get_command());
    process_buffered_command(gcode);
    return;
}
```

**New Sequence (Correct):**
```
G41 D10 → BYPASS_BUFFER, activates compensation, clears buffer
G1 X10 Y0 → buffered (count=1)
G1 X10 Y10 → buffered (count=2)
G40 → BYPASS_BUFFER, sets is_flushing=true, loops calling get_compensated_gcode() to drain
→ Both buffered commands output, compensation deactivates
```

**Why This Was The Critical Fix:**
- G40 must execute immediately to set flushing flag
- G41/G42 must execute immediately to activate compensation before next moves buffer
- Control commands for a system cannot be processed by that system
- This architectural principle applies to any command-processing buffer

**Result:**
- All commands flush completely on G40
- Buffer operations are now predictable and deterministic
- User confirmed: "tested out perfectly"

**Status:** ✅ Resolved in firmware_v2.4_PHASE1_G40_BYPASS.bin

**Key Architectural Learning:**
This fix revealed a fundamental principle: **Control commands that manage a processing system must bypass that system**. In this case, G40/G41/G42 control the buffer, so they must never be buffered themselves. This pattern likely applies to other control systems (e.g., feed override commands shouldn't wait in motion planner queue).

---

## Current State (Updated: April 1, 2026)

### Phase 1: Buffer Infrastructure (✅ COMPLETE - Feb 19, 2026)

**Firmware Version:** firmware_v2.4_PHASE1_G40_BYPASS.bin (448,200 bytes)  
**Status:** ✅ FULLY VALIDATED - All tests passed

**Working Features:**
✅ Circular buffer system (10 slots, head/tail pointers)
✅ Always-on buffering (all commands buffer regardless of compensation state)
✅ Pass-through mode (when comp OFF, commands bypass buffer entirely)
✅ Lookahead requirement (holds until 3+ commands before outputting)
✅ Flushing mode (G40 sets flag to drain buffer)
✅ Control command bypass (G40/G41/G42 never enter buffer)
✅ Position tracking (uncompensated_position and compensated_position arrays)
✅ Null pointer validation with auto-recovery
✅ G40/G41/G42 G-code parsing
✅ D word diameter reading
✅ Compensation state management
✅ Debug output system with multiple prefix types
✅ Works with straight lines (G0/G1)
✅ Works with arcs (G2/G3)
✅ No data loss, duplication, or corruption
✅ System stable under hardware testing

**Validated Test Scenarios:**
- Multiple G1 moves with G41/G42 active
- G40 flushing after multiple buffered commands
- G41/G42 activation (clears buffer, starts fresh)
- Mixed G0/G1/G2/G3 commands
- Rapid compensation on/off cycling
- Commands execute correctly uncompensated when comp OFF
- Commands buffer correctly when comp ON

### Phase 2: Simple Perpendicular Offset (✅ COMPLETE - Feb 23 → Mar 15, 2026)

**Firmware Version:** firmware_v2.7_PHASE2_PERPENDICULAR_OFFSET.bin (451,608 bytes)  
**Status:** ✅ COMPLETE - Perpendicular offset working, corner artifacts identified

**Implemented Features:**
✅ `calculate_perpendicular_offset()` function
✅ Apply offset to straight line endpoints
✅ Tested with square path (offset applied successfully)
✅ Offset magnitude correct by radius
✅ Debug output shows compensated coordinates
✅ G41 (LEFT) compensation working
✅ G42 (RIGHT) compensation working

**Known Limitation - Corner Diagonal Artifacts:**
❌ Offset calculated independently per segment (entering direction only)
❌ Gaps at corners manifest as diagonal shortcuts
❌ Example: Corner at (50,0)→(50,50) shows comp[1]=(45,0) then comp[2]=(50,45)
❌ Tool traces diagonal from (45,0) to (50,45) instead of clean corner

**Cause:** Algorithm offset each intermediate point perpendicular to the segment leading INTO it, without considering the segment leaving FROM it. Geometrically incomplete.

**Resolution:** Phase 3 implements true corner intersection geometry.

---

### Phase 3: Geometrically Correct Corner Intersection (✅ COMPLETE - April 1, 2026)

**Firmware Version:** firmware_v2.7_PHASE3_CORNER_INTERSECTION_20260401.bin (450,816 bytes)  
**Status:** ✅ FULLY VALIDATED - Perfect corners achieved, zero diagonal artifacts

**Achievement:**
✅ Eliminated corner diagonal artifacts completely
✅ Unified A-B-C corner intersection algorithm
✅ Handles three cases (onset/corner/terminal) with single formula
✅ Code reduced from 120+ to ~50 lines (cleaner, more maintainable)
✅ Machine testing confirms perfect corner geometry
✅ Kivy log verification shows correct miter joins

**Algorithm Summary:**
- **Unified approach:** Single `calculate_corner_intersection()` for all three compute cases
- **5-step geometry:** Direction vectors → perpendicular normals → offset lines → intersection
- **Line-line intersection:** Parametric equation solving with epsilon tolerance
- **Degenerate handling:** Onset (A=B) and Terminal (B=C) reduce to perpendicular offsets
- **Implementation:** ~70 lines in calculate_corner_intersection(), 4 lines each in compute_and_output() and compute_terminal_output()

**Test Results (from kivy_26-04-01_0.txt):**
```
Outer Square Compensation:
  comp[0] = (0,5)     ← Onset perpendicular offset
  comp[1] = (45,5)    ← Miter-joined corner intersection ✅
  comp[2] = (45,45)   ← Miter-joined corner intersection ✅
  comp[0] = (5,45)    ← Miter-joined corner intersection ✅

"That last build executed with no diagonals and made perfect corners" - Operator Confirmation
```

**Verification Method:**
- Square-in-square test program (double rectangular cut)
- Both G41 (LEFT) and G42 (RIGHT) modes
- Machine generated compensated coordinates extracted from Kivy log
- Perfect 90° corners with no diagonal shortcuts
- Dimension verification passed
- No tool chatter, gouge, or hesitation

**Code Changes:**
- **CompensationPreprocessor.h:** Added `calculate_corner_intersection()` declaration
- **CompensationPreprocessor.cpp:**
  - Refactored `compute_and_output()` (120→50 lines, eliminated branching)
  - Updated `compute_terminal_output()` (now calls unified function)
  - Implemented `calculate_corner_intersection()` (70 lines)
  - Removed duplicate perpendicular offset calculations

**Comparison to Phase 2:**
| Aspect | Phase 2 | Phase 3 | Result |
|--------|---------|---------|--------|
| Corner at (50,0)→(50,50) | comp[1]=(45,0), comp[2]=(50,45) | comp[1]=(45,5), comp[2]=(45,45) | **Perfect miter join!** |
| Code branches | 2 separate paths | 1 unified path | **-58% code** |
| Degenerate support | None (first point special case) | Built-in (A=B, B=C) | **Elegant** |
| Corner quality | Diagonal artifacts ❌ | Zero artifacts ✅ | **Problem solved** |

**For Complete Phase 3 Documentation:** See [PHASE3_CORNER_INTERSECTION_MILESTONE.md](./PHASE3_CORNER_INTERSECTION_MILESTONE.md)

---

### Future Phases:

**Phase 4: Modal G-code Preservation (⏸️ FUTURE - Priority: MEDIUM)**
⏸️ Currently: Outputs hardcoded "G1 X Y Z"
⏸️ Enhancement: Preserve input motion mode (G0 rapid vs G1 feed)
⏸️ Enhancement: Preserve feed rate (F word)
⏸️ Impact: Proper rapid vs feed moves, machine velocity control

**Phase 5: Advanced Features (⏸️ FUTURE - Priority: LOW)**
⏸️ Arc-to-arc transitions (optimize corner handling for arc sequences)
⏸️ Lead-in/lead-out moves (approach/retract paths)
⏸️ Multi-tool strategies (tool library optimization)
⏸️ CNC simulation tool (visual verification)
⏸️ Inside vs outside corner detection
⏸️ Intersection point calculation
⏸️ Eliminate gaps at corners

**Phase 4: Lead-In Moves (⏸️ FUTURE)**
⏸️ Lead-in move generation
⏸️ Use buffered moves to determine entry direction
⏸️ Eliminate diagonal on first compensated move

**Phase 5: Arc Compensation (⏸️ FUTURE)**
⏸️ Arc endpoint offset calculation
⏸️ Arc center (I/J/K) offset modification
⏸️ Arc-to-line and line-to-arc transitions

**Future Enhancements:**
⏸️ Lead-out moves
⏸️ G-code validation/error checking
⏸️ Tool table integration (currently using explicit D word)
⏸️ Advanced corner strategies (arc insertion for tight corners)

---

## Phase 1 Test Results (February 19, 2026)

### Test Configuration
**Firmware:** firmware_v2.4_PHASE1_G40_BYPASS.bin  
**Size:** 448,200 bytes  
**Hardware:** Carvera CNC with LPC1768 controller  
**Tester:** User (Matth)  

### Test Scenarios Executed

**Test 1: Basic Buffering Without Compensation**
```gcode
G0 X0 Y0
G1 X10 Y0
G1 X10 Y10
G1 X0 Y10
```
**Result:** ✅ PASS
- All commands executed in order
- No buffering (pass-through mode active)
- No errors or crashes

**Test 2: Buffering With Compensation Active**
```gcode
G41 D10
G1 X10 Y0
G1 X10 Y10
G1 X0 Y10
```
**Result:** ✅ PASS
- G41 activated compensation and cleared buffer
- Commands buffered correctly (count showed 1, 2, 3)
- Commands output when lookahead satisfied
- Buffer maintained 2-3 commands in steady state

**Test 3: G40 Flushing**
```gcode
G41 D10
G1 X10 Y0
G1 X10 Y10
G1 X0 Y10
G40
```
**Result:** ✅ PASS
- G40 bypassed buffer (executed immediately)
- Flushing flag set
- All remaining buffered commands output
- Buffer emptied completely
- No commands lost

**Test 4: Multiple G41/G42/G40 Cycles**
```gcode
G41 D10
G1 X10 Y0
G40
G42 D6
G1 X10 Y10
G40
```
**Result:** ✅ PASS
- Each G41/G42 cleared buffer and started fresh
- Each G40 flushed remaining commands
- No interference between cycles
- System stable throughout

**Test 5: Arc Commands (G2/G3)**
```gcode
G41 D10
G2 X10 Y0 I5 J0
G40
```
**Result:** ✅ PASS
- Arc commands buffered like straight lines
- No special handling needed for arcs in Phase 1
- Arcs output correctly

**Test 6: Rapid Cycling**
```gcode
G41 D10
G1 X10 Y0
G40
G41 D10
G1 X0 Y0
G40
```
**Result:** ✅ PASS
- System handled rapid on/off cycling
- No buffer corruption
- No pointer errors
- System remained stable

### Performance Metrics

**Buffer Operations:**
- Buffer size: 10 slots
- Typical occupancy: 2-3 commands (lookahead requirement)
- Maximum observed: 4 commands
- No buffer overflows encountered

**Timing:**
- No noticeable latency added
- Motion remained smooth
- No stuttering or pauses

**Memory:**
- No memory leaks detected
- System stable over extended operation

**Reliability:**
- No hard limit errors
- No coordinate corruption
- No crashes or reboots
- No data loss or duplication

### User Feedback

**Quote:** "Ok there we go. That build tested out perfectly. I think that we finally passed the test for the buffer working as expected"

**Interpretation:**
- Buffer infrastructure is fully functional
- Ready to proceed to Phase 2 (adding actual compensation)
- Foundation is solid for building additional features

### Lessons Learned

1. **Isolation Testing is Effective**
   - Testing buffer alone (without compensation) made debugging tractable
   - Could clearly identify buffer-specific issues vs compensation math issues
   - Incremental approach prevented "debugging everything at once"

2. **Pass-Through Mode is Essential**
   - Prevents stale data from affecting uncompensated moves
   - Simpler than constantly clearing/resetting buffer
   - Only use buffer when actually needed

3. **Control Command Bypass is Mandatory**
   - G40/G41/G42 must execute immediately, cannot be buffered
   - Control commands for a system cannot be processed by that system
   - This principle likely applies elsewhere in CNC control architecture

4. **Lookahead Requirement Simulates Real Needs**
   - Requiring 3+ commands prevents premature draining
   - Tests the buffer's ability to hold and sequence commands
   - Will be essential for corner intersection calculations in Phase 3

5. **Extensive Debug Output Accelerates Debugging**
   - Multiple prefix types (`>>PASSTHROUGH:`, `>>BUFFER:`, `>>BYPASS_BUFFER:`, etc.)
   - Visibility into buffer state (count, head, tail)
   - Slot content inspection helped identify garbage data issue
   - Will keep debug output for Phase 2+ development

### Phase 1 Sign-Off

**Status:** ✅ COMPLETE  
**Date:** February 19, 2026  
**Decision:** Proceed to Phase 2 (Simple Perpendicular Offset)  
**Confidence Level:** HIGH - All validation criteria met  

---

## Build Status

**Last Successful Build:** Prior to latest changes
**Current Build Status:** ⏳ Awaiting rebuild

**Build Commands:**
```bash
cd C:\Users\Matth\Desktop\Personal\equipment_and_tools\Carvera_CNC\Carvera_Community_Firmware
.\BuildShell.cmd
make clean
make all AXIS=5 PAXIS=3 CNC=1
```

**Output Binary:** `LPC1768/main.bin` → rename to `firmware_Exp_CutComp.bin`

---

## Next Steps

### Immediate Priorities:
1. **Recompile and Test** - Verify latest zigzag fixes work correctly
2. **Validate Coordinate Spaces** - Ensure compensation works in correct coordinate system
3. **Single Line Test** - Execute simple G1 X50 Y0 with G41 and verify smooth motion
4. **Full Test Suite** - Run complete TEST_phase1_G4x_D_word.cnc program

### Short Term:
1. Integrate corner handling from corner_handling.cpp
2. Implement proper lookahead for corner detection
3. Add lead-in/lead-out moves
4. Test with 50x50mm rectangle (Tests 3-4)
5. Test with arcs (Tests 5-6)

### Long Term:
1. Add tool table integration (read diameter from tool table)
2. Implement G-code validation
3. Add error handling for invalid compensation scenarios
4. Optimize performance
5. Add comprehensive test suite
6. Document usage in user manual

---

## Technical Notes

### Coordinate Systems
- **Machine Coordinates:** Absolute position in machine space
- **Workpiece Coordinates:** Position relative to current work coordinate system (G54-G59)
- **Compensation Space:** Where offset calculations occur (should be workpiece coordinates)

### Normal Vector Calculation
For a move from point A to point B:
1. Calculate direction vector: `[dx, dy] = B - A`
2. Normalize: `[ux, uy] = [dx, dy] / length`
3. Calculate normal (perpendicular):
   - **Left (G41):** Rotate 90° CCW: `[nx, ny] = [-uy, ux]`
   - **Right (G42):** Rotate 90° CW: `[nx, ny] = [uy, -ux]`
4. Apply offset: `compensated_point = original_point + [nx, ny] * radius`

### Move Segmentation
The firmware's `mm_per_line_segment` setting (default 5mm) breaks long moves into smaller segments. This affects compensation because:
- Each segment is processed independently
- Offset direction must remain consistent across segments
- Short segments need special handling to avoid oscillation

### Debug Output Patterns
Look for these patterns in debug output:
- `DBG:CompPrep:` - CompensationPreprocessor messages
- `DBG:Robot:` - Robot class compensation messages
- `DBG:CutterComp:` - Detailed compensation calculations
- Move coordinates should show consistent offset in Y direction for X-axis moves with compensation

---

## References

### G-code Standards
- **G40:** Cancel cutter compensation
- **G41:** Enable left compensation (tool to left of programmed path)
- **G42:** Enable right compensation (tool to right of programmed path)
- **D word:** Tool diameter specification (in mm)

### Key Files for Reference
- `src/modules/robot/Robot.cpp` - Main robot control, move processing
- `src/modules/robot/Planner.cpp` - Motion planning
- `src/modules/robot/Conveyor.cpp` - Move queue management
- `src/config.default` - Default configuration including `mm_per_line_segment`

### Helpful Debug Commands
```gcode
M114    ; Report current position
M503    ; Report settings
G54     ; Select work coordinate system 1
G92 X0 Y0 Z0  ; Set current position as origin
```

---

## Known Limitations and Edge Cases

### Current Limitations

1. **G-code Restrictions**
   - Cannot enable compensation during a move (must be activated while stationary or in G0)
   - Does not yet validate if compensation can be safely applied
   - No error handling for tool radius larger than inside corner radius

2. **Arc Handling**
   - Arc compensation uses I,J offset scaling method
   - Does not yet validate arc feasibility with compensation applied
   - Helical arcs (with Z movement) not yet tested

3. **Coordinate Systems**
   - Compensation applied in current workpiece coordinate system (G54-G59)
   - G92 offsets are applied before compensation
   - Coordinate system changes during active compensation not fully tested

4. **Tool Table Integration**
   - Currently requires explicit D word in G41/G42 command
   - Does not read from tool table (future enhancement)
   - No tool diameter validation against machine geometry

5. **Segmentation Interaction**
   - `mm_per_line_segment` setting causes moves to be broken into smaller segments
   - Current implementation handles this but may affect performance
   - Very small segments (< 0.1mm) are filtered to maintain consistency

### Edge Cases to Test

1. **Zero-Length Moves**
   - Moves where start = end position
   - Currently handled by inheriting previous direction

2. **Collinear Moves**
   - Three or more moves in the same direction
   - Should maintain consistent offset without corner calculation

3. **Reversing Direction (180° corners)**
   - Moving forward then immediately backward
   - May require special handling or error reporting

4. **Very Sharp Inside Corners**
   - When tool radius exceeds corner radius
   - May require arc insertion or error reporting
   - Not yet implemented

5. **Compensation During Arc**
   - Enabling G41/G42 while already on an arc move
   - Current implementation may not handle gracefully

6. **Mixed Coordinate Systems**
   - G53 (machine coordinates) with active compensation
   - G2/G3 arcs in incremental mode (G91)
   - Requires careful testing

### Future Enhancements Needed

1. **Error Detection and Reporting**
   - Tool too large for inside corner
   - Compensation enabled during invalid conditions
   - Self-intersecting offset paths

2. **Advanced Corner Strategies**
   - Arc insertion at sharp inside corners
   - Chamfer vs. arc options
   - User-configurable corner behavior

3. **Performance Optimization**
   - Reduce debug output in production builds
   - Optimize vector calculations
   - Minimize move buffer allocations

4. **Tool Table Integration**
   - Read tool diameter from tool table
   - Support tool offsets in addition to radius
   - Validate tools before use

---

## Debugging Guide

### Essential Debug Output Patterns

When troubleshooting compensation issues, look for these debug messages:

**Compensation Activation:**
```
DBG:Robot: Setting compensation side=1 radius=3.000
DBG:CompPrep: Enabling compensation side=1 diameter=6.000
DBG:CompPrep: Enabled LEFT compensation, radius=3.000
```

**Move Processing:**
```
DBG:CompPrep: Processing move [0.000,0.000] -> [50.000,0.000] (comp_side=1)
DBG:CompPrep: Buffered move #0 [0.000,0.000] -> [50.000,0.000]
DBG:CompPrep: Move vector [1.000,0.000] normal [-0.000,1.000]
DBG:CompPrep: Applied offset [50.000,0.000] -> [50.000,3.000]
```

**Zigzag Symptoms:**
```
DBG:CutterComp: Applied offset 3.000 -> [-174.000,-121.000]  // Offset applied
DBG:CutterComp: Move modified from [-169.000,-124.000] to [-169.000,-124.000]  // No offset!
DBG:CutterComp: Applied offset 3.000 -> [-164.000,-121.000]  // Offset applied
```
This alternating pattern indicates segmentation handling issues.

**Coordinate Space Issues:**
```
DBG:CompPrep: Processing move [-179.000,-124.000] -> [-129.000,-124.000]
```
Large negative coordinates when you commanded X0 Y0 indicate working in machine coordinates (includes G54/G92 offsets).

### Common Problems and Diagnostics

**Problem: No compensation applied**
- Check: `compensation_active` flag set?
- Check: `comp_side` not NONE?
- Check: `comp_radius` > 0?
- Check: Is `preprocess_move()` being called?

**Problem: Wrong offset direction**
- Check: G41 vs G42 setting
- Check: Normal vector calculation (LEFT = CCW rotation, RIGHT = CW rotation)
- Check: Work coordinate system orientation

**Problem: Zigzag motion**
- Check: Move direction caching working?
- Check: Short moves using previous direction?
- Check: `mm_per_line_segment` setting (try increasing to reduce segmentation)

**Problem: Corners are wrong**
- Check: Lookahead buffer has enough moves?
- Check: Corner detection working?
- Check: Intersection calculation correct?

### Manual Testing Procedure

**Basic Single-Line Test:**
```gcode
G90 G21          ; Absolute mode, metric
G0 X0 Y0 Z5      ; Safe position
G41 D6           ; Enable left compensation, 6mm tool
G1 F1000 X50 Y0  ; Move 50mm in X
G40              ; Cancel compensation
M114             ; Report position - should show Y=3.000
```

**Expected Behavior:**
- Machine moves to X50, Y3 (3mm offset to the left)
- Motion should be smooth, no zigzag
- Debug output shows consistent Y=3.000 throughout

**Rectangle Test (Outside Corners):**
```gcode
G90 G21
G0 X0 Y0 Z5
G41 D6
G1 F1000 X50 Y0
G1 X50 Y50
G1 X0 Y50
G1 X0 Y0
G40
```

**Expected Behavior:**
- Tool traces a 50x50mm square
- 3mm offset maintained around all corners
- Corners should be sharp (intersections calculated correctly)

### Configuration Settings Impact

**`mm_per_line_segment`** (default: 5mm)
- Lower values = more segments = more preprocessing overhead
- Higher values = fewer segments = smoother compensation
- For testing, try setting to 50mm to minimize segmentation:
  ```
  mm_per_line_segment 50
  ```

**`default_feed_rate`** 
- Use slower feed rates for testing (100-300 mm/min)
- Easier to observe motion and diagnose issues
- Production speeds can be higher once validated

**Soft Limits**
- Ensure soft limits account for compensation offset
- Tool path + radius must stay within limits
- Check x_min, x_max, y_min, y_max settings

---

## Development Environment Notes

### Build System Quirks

**ARM GCC Toolchain Path:**
- Must be set up via `BuildShell.cmd` before building
- Located in: `gcc-arm-none-eabi/bin/`
- Version: arm-none-eabi-gcc 4.8 (2014-q1-update)

**Make System:**
- Uses custom makefiles in `build/` directory
- Windows: Uses `build/win32/make.exe`
- Must specify AXIS=5 PAXIS=3 CNC=1 for Carvera

**Common Build Errors:**
- "arm-none-eabi-gcc not found" → Run BuildShell.cmd first
- "comparison always false" → Enum comparison issue (use != instead of < or >)
- "function definition not allowed here" → Missing closing brace or scope issue
- "identifier undefined" → Check forward declarations and includes

### Code Style and Conventions

**Naming Conventions:**
- Class names: PascalCase (CompensationPreprocessor)
- Methods: snake_case (preprocess_move)
- Member variables: snake_case (comp_side, comp_radius)
- Constants: UPPER_SNAKE_CASE (LOOKAHEAD_SIZE)

**Debug Output:**
- Use `THEKERNEL->streams->printf()` for debug messages
- Prefix with `DBG:ModuleName:` for easy filtering
- Include relevant values (coordinates, vectors, flags)
- Use %.3f for float formatting (3 decimal places)

**Memory Management:**
- Use `new` in constructor, `delete` in destructor
- CompensationPreprocessor owned by Robot class
- Move buffer uses std::vector for automatic management
- Avoid dynamic allocation in real-time paths

### Testing on Hardware

**Safety Precautions:**
1. Always start with Z axis raised (Z5 or higher)
2. Use G0 (rapid) to move to start position
3. Set conservative feed rates (F100-F300) for initial tests
4. Have E-stop accessible
5. Watch for unexpected motion - be ready to stop

**Test Progression:**
1. Start with air cutting (Z above workpiece)
2. Verify coordinates in debug output match expectations
3. Test single line moves first
4. Then test corners (rectangle)
5. Finally test arcs
6. Only cut material once all tests pass in air

**Reverting to Safe State:**
- If compensation behaves unexpectedly, send `G40` immediately
- `M999` resets firmware (emergency)
- Power cycle machine if firmware becomes unresponsive
- Keep backup firmware on SD card

---

## Implementation History and Context

### Initial Approach (Abandoned)

**Direct Offsetting in append_milestone():**
- First attempt was to offset coordinates directly when adding moves to planner
- Problem: No visibility into next move at that point
- Result: Corners were incorrect, gaps and overlaps

**Learning:** Must intercept moves earlier, before they reach the planner.

### Second Approach (Abandoned)

**Post-Processing in Planner:**
- Attempted to buffer moves in the planner and adjust there
- Problem: Planner deals with step timing, not geometry
- Problem: Too late to modify coordinates safely
- Result: Timing issues, unpredictable behavior

**Learning:** Geometry calculations must happen before motion planning.

### Current Approach (Active)

**Preprocessing Before Planner:**
- Intercept in `Robot::on_gcode_received()` after coordinate transformation
- Buffer moves in CompensationPreprocessor
- Calculate offsets with lookahead
- Modify target coordinates before passing to planner
- Result: Clean integration, correct geometry

**Learning:** This is the correct architectural point for compensation.

### Key Design Decisions

**Decision: Use Separate Preprocessor Class**
- Rationale: Keeps Robot.cpp manageable, clear separation of concerns
- Alternative: Implement directly in Robot class (would be messy)
- Result: Clean, testable, maintainable code

**Decision: Store Direction Vectors in Move Struct**
- Rationale: Avoid recalculating unit vectors repeatedly
- Alternative: Calculate on-demand (more CPU overhead)
- Result: Faster, fixes segmentation consistency issues

**Decision: Filter Short Moves (< 0.1mm)**
- Rationale: Segmentation creates tiny moves that cause oscillation
- Alternative: Process all moves (causes zigzag)
- Result: Smooth motion on segmented lines

**Decision: Use D Word for Diameter (for now)**
- Rationale: Simple, explicit, easy to test
- Alternative: Tool table integration (more complex, future work)
- Result: Working implementation, can enhance later

### Collaboration Notes

**This implementation was developed through iterative problem-solving:**
1. Started with basic per-move offsetting
2. Discovered corner handling requirements
3. Researched industry standards (Fanuc, Haas, etc.)
4. Designed preprocessor architecture
5. Implemented basic linear compensation
6. Debugged coordinate space issues
7. Fixed enum comparison warnings
8. Solved zigzag problem through direction caching
9. Currently: Testing and refinement

**Key Insights Gained:**
- Preprocessing is not optional - it's fundamental to correct compensation
- Move segmentation has major impact on compensation logic
- Direction vector caching is essential for consistency
- Debug output is critical for diagnosing geometric issues
- Industry standards exist for good reasons - follow them

---

## Revision History

**November 3-6, 2025:**
- Initial implementation of compensation framework
- Created CompensationTypes.h, CompensationPreprocessor.h/.cpp
- Integrated into Robot class
- Fixed enum comparison warnings
- Fixed compilation errors with missing braces
- Implemented move direction caching
- Added short move handling to prevent zigzag
- Enhanced debug output
- Documented all changes and current state
- Added comprehensive preprocessing rationale
- Documented known limitations and edge cases
- Created debugging guide and testing procedures

---

## Contact & Collaboration

This implementation is part of the Carvera Community Firmware project:
- **Repository:** https://github.com/Carvera-Community/Carvera_Community_Firmware
- **Branch:** 3dtoolsetter
- **Related PR:** #133 (3D Tool Setter)

For questions or contributions related to cutter compensation, refer to this document and the test files in `tests/TEST_basic_cutter_comp/`.

---

*Document Last Updated: November 6, 2025*
