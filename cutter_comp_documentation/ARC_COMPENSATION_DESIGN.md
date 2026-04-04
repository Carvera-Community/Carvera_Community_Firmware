# Arc Compensation: Design Proposal
## Pre-Implementation Review Document — April 4, 2026

---

## Executive Summary

This document proposes the design for native G2/G3 arc support inside the
dual-ring compensation preprocessor.  It is a planning document only.
No code is changed until the design has been reviewed and agreed.

**What this covers**

1. How the current pipeline receives and processes arcs today
2. Why the current architecture cannot pass arcs through with geometric
   correctness without changes
3. The proposed schema extension to `UncompPoint` and `CompPoint`
4. The arc center-offset compensation math
5. Transition strategy for line→arc, arc→line, and arc→arc junctions
6. The separate "segment too small to compensate" phase proposal
7. Hard-error policy for unsupported arc dialects under G41/G42
8. Testing strategy and acceptance gates

**Scope**

| Topic | Phase | Status |
|---|---|---|
| G17 plane (XY) arc compensation | Arc Phase 1 | Proposed here |
| Helical arcs (G2/G3 with Z change) | Arc Phase 1 | Proposed here |
| G18 / G19 plane compensation | Arc Phase 2 | Deferred |
| R-format arc compensation | Arc Phase 2 | Deferred (hard error in phase 1) |
| Segment-skip for too-small moves | Separate Phase | High-priority design note only |

---

## 1. How Arcs Flow Through the Current Pipeline

### 1.1 Without Compensation

When compensation is not active the arc path is straightforward:

```
on_gcode_received()
    → process_move()
        → compute_arc()          (extracts radius from I/J)
            → append_arc()       (segments arc into N milestones)
                → append_milestone() × N   (each goes to planner)
```

The key detail is that `append_arc()` converts the arc into line segments
*before* anything goes to the planner.  The arc center offsets I and J are
consumed and discarded inside `compute_arc()`.  After this point, only XYZ
positions survive into the execution queue.

### 1.2 With Compensation Active Today

When G41 or G42 is active, `on_gcode_received()` intercepts the incoming
Gcode before `process_move()` is called:

```
on_gcode_received()
    → buffer_gcode(gcode)       (compensation preprocessor takes over)
        → stores gcode.X, gcode.Y, gcode.Z in uncomp_ring
        → I/J/K are silently ignored
        → output is always "G1 X Y Z"
```

The consequence is that a G2 or G3 command today becomes a single linear
move to the arc endpoint.  The entire curvature is lost.

### 1.3 Why Intercepting Before Segmentation Is Correct

The compensation must see the arc as a whole before segmentation because:

- Compensation offsets the arc radius, not individual chord endpoints.
- Compensating each chord independently introduces artificial corners at
  every segment boundary.
- The corrected arc center and I/J offsets must be computed once per arc
  command, then the corrected arc goes on to segmentation.

The proposed design keeps this interception point and adds arc parameter
preservation to the ring buffers.

### 1.4 Current Modal Handling (G, F, S) in Existing Firmware

This section documents current behavior before arc-phase changes.

1. **G motion mode (G0/G1/G2/G3)**
    - In `process_buffered_command()`, the incoming G code is mapped to motion
       mode (`SEEK`, `LINEAR`, `CW_ARC`, `CCW_ARC`) and then executed through
       `process_move()`.
    - In the current compensation path, reconstructed output is hardcoded as
       `G1 X... Y... Z...`, so original G0/G2/G3 intent is not preserved.

2. **F feedrate**
    - In `process_move()`, if a command has `F`, firmware updates modal
       `feed_rate` (or `seek_rate` for G0).
    - Current compensated outputs do not emit `F`, so an explicit `F` present
       on buffered input can be lost unless it was already applied earlier in
       non-compensated execution context.

3. **S value (motion-modal S in this firmware path)**
    - In `process_move()`, when `S` appears on G0/G1/G2/G3, `s_value` is
       updated and treated as modal for subsequent motion behavior in modules
       that consume it.
    - Current compensated reconstruction does not emit `S`, so explicit S words
       present on compensated input are not preserved.

Design implication:

- Arc-phase work must restore semantic preservation for motion-mode and modal
   state-bearing words relevant to motion reconstruction, with deterministic
   behavior through the dual-ring delay.

---

## 2. The Arc Compensation Geometry

### 2.1 What Changes When a Radius Is Compensated

Without compensation a tool center moves on the programmed arc.  With
compensation the tool *edge* must ride the programmed arc, so the tool
center must move on a larger or smaller concentric arc.

```
Programmed arc center:   C  = (start_x + I,  start_y + J)
Programmed arc radius:   r  = hypot(I, J)

Compensated arc radius:  r' = r ± comp_radius

  G41 (LEFT),  outside convex arc  →  r' = r + comp_radius
  G41 (LEFT),  inside concave arc  →  r' = r − comp_radius
  G42 (RIGHT), outside convex arc  →  r' = r − comp_radius
  G42 (RIGHT), inside concave arc  →  r' = r + comp_radius
```

The arc center **does not move**.  Only the radius changes.  This means the
compensated I/J offsets from the new start point to the same center can be
recomputed once the new start point is known.

### 2.2 Sign Convention for Convex vs Concave

Whether an arc is convex or concave relative to the tool side depends on
the combination of compensation direction (G41/G42) and arc direction
(G2/G3):

| Compensation | Arc direction | Tool side | Result |
|---|---|---|---|
| G41 (LEFT)  | G2 (CW)  | Left of motion = inside arc  | r' = r − comp_radius |
| G41 (LEFT)  | G3 (CCW) | Left of motion = outside arc | r' = r + comp_radius |
| G42 (RIGHT) | G2 (CW)  | Right of motion = outside arc| r' = r + comp_radius |
| G42 (RIGHT) | G3 (CCW) | Right of motion = inside arc | r' = r − comp_radius |

Simplified: when the compensation side is the same as the arc interior,
subtract; when the compensation side faces the arc exterior, add.

### 2.3 The 4-Step Arc Compensation Algorithm

Given an arc command buffered in `uncomp_ring[b]`:

**Step 1 — Recover Arc Center**
```
C.x = start.x + I
C.y = start.y + J
```
where `start` is the position stored in `uncomp_ring[b-1]` (the previous
point, which is the arc entry point).

**Step 2 — Compute New Radius**
```
r  = hypotf(I, J)
r' = r + sign * comp_radius
```
where `sign` is +1 or −1 from the convex/concave table above.

**Step 3 — Compute New Start Point**

The new start point is the intersection of the previous compensated move
with the offset arc.  For the onset and for line→arc transitions this is
the existing corner intersection algorithm (Phase 3), which gives us a
compensated XY for the arc's entry point.  This is already computed by
`compute_and_output()`.

**Step 4 — Recompute I/J from New Start Point to Same Center**
```
new_I = C.x − comp_start.x
new_J = C.y − comp_start.y
```

The compensated G2/G3 command to emit is:
```
G2/G3 X<comp_end.x> Y<comp_end.y> Z<comp_end.z> I<new_I> J<new_J> F<feedrate>
```

The arc end point (`comp_end`) requires its own separate intersection
step and is discussed in Section 4 (Transitions).

### 2.4 Helical Arc (Z Change)

When a G2/G3 also has a Z word, the arc is helical.  The XY compensation
applies identically.  The Z component is passed through without modification,
exactly as it is done today for linear moves — Z is always the programmed
value at the arc endpoint.

---

## 3. Schema Extension: Adding Arc Metadata to the Ring Buffers

### 3.1 Current `UncompPoint` Schema

```cpp
struct UncompPoint {
    float x, y, z;          // Endpoint only
};
```

### 3.2 Proposed `UncompPoint` Schema

```cpp
struct UncompPoint {
    float x, y, z;          // Endpoint (XYZ, WCS)
    uint8_t motion_mode;    // 0=rapid, 1=linear, 2=CW arc, 3=CCW arc
    float ij[2];             // I, J offsets when motion_mode == 2 or 3 (else 0)
   float feedrate;          // Optional: F value seen on this command
   bool  has_feedrate;      // Optional: true if this command carried explicit F
};
```

**Memory impact:**  
Current: 12 bytes per slot × 3 slots = 36 bytes  
Proposed: 24 bytes per slot × 3 slots = 72 bytes  
Net increase: +36 bytes — negligible on LPC1768.

### 3.3 Feedrate in Schema: Steel-Man Case and Alternative

The strongest argument for keeping feedrate in schema is not geometric
accuracy; it is deterministic execution semantics and future-proofing:

1. **Deterministic reconstruction**
   The preprocessor reconstructs Gcode strings and may emit extra moves in
   future phases (segment-skip merge, lead-in/out). Capturing explicit F per
   buffered command guarantees reconstructed output preserves exactly what the
   source command intended, independent of later modal state changes.
2. **Temporal decoupling safety**
   The dual-ring pipeline introduces delay between intake and output. If modal
   feed is read from external runtime state at output time, feed semantics can
   drift when upstream commands alter modal state before delayed emission.
3. **Debug and traceability**
   When investigating path anomalies, having explicit buffered feed context
   avoids ambiguity about whether an output move inherited or overrode modal F.
4. **Low cost**
    The per-slot storage increase is small today, but this cost is cumulative
    across features and cannot be hand-waved as universally low impact.
    Design purity requires tracking this budget explicitly as buffer depth and
    special-case handling grow.

Memory scaling guardrail:

- Feature fields scale approximately with `bytes_per_slot * BUFFER_SIZE`.
- Growth is linear with buffer size, but cumulative feature additions can still
   become significant on constrained targets.
- Every field added to `UncompPoint` should be justified against a measurable
   semantic need or testability gain.

That said, a valid performance-first alternative is:

- **Mode+geometry only schema (no feed fields)**
  Keep feed behavior exactly as current modal handling in Robot path and avoid
  storing F in `UncompPoint`.

Recommended compromise for phase 1:

- Keep the fields in schema but do not emit extra F words unless explicitly
  present on input. This preserves determinism with minimal output bloat.

### 3.4 Proposed `CompPoint` Schema

No change required.  `CompPoint` stores the compensated endpoint and the
reconstructed Gcode pointer, which already accommodates the longer output
string.

### 3.5 Gcode Output String Length

The current buffer for output string reconstruction is:
```cpp
char gcode_str[128];
snprintf(gcode_str, sizeof(gcode_str), "G1 X%.3f Y%.3f Z%.3f", ...);
```

The proposed arc output format is:
```
G2 X12.345 Y-6.789 Z1.000 I3.456 J-2.345 F1000.000
```
This fits well within 128 bytes (worst case ~55 characters).  No buffer
resize needed.

---

## 4. Transition Strategy

The compensation preprocessor maintains a lookahead window of three points:
A (previous), B (current), C (next).  The same window is used for arcs,
but the intersection logic at point B changes depending on what type of
move leads in and what leads out.

### 4.1 Line → Line (Existing, Unchanged)

Uses Phase 3 miter intersection.  Not changed by arc support.

### 4.2 Line → Arc

Point B is the first point of the arc.  The entry direction (from A to B)
comes from the outgoing line.  The exit direction (leaving B along the arc)
is the tangent to the arc at its start point:

```
Tangent at arc start = perpendicular to the radius vector at the start

radius_vec = normalize(start - center)   // Vector from center to start
tangent    = rotate_90(radius_vec, arc_direction)
```

The corner at B is computed using:
- `u_in`  = direction of the incoming line (A → B)
- `u_out` = arc tangent direction at the arc's compensated start

The Phase 3 miter intersection algorithm applies unchanged once these
two direction vectors are available.

### 4.3 Arc → Line

Point B is the last point of the arc.  The entry direction follows the arc
tangent at its endpoint.  The exit direction is the line direction:

```
tangent_out = rotate_90(normalize(end - center), arc_direction)
```

Same miter intersection with:
- `u_in`  = arc tangent at compensated arc end
- `u_out` = direction of outgoing line

### 4.4 Arc → Arc

Point B is the junction between two arcs.  Both entry and exit directions
are arc tangents:

- `u_in`  = tangent of arc A at its endpoint (= point B)
- `u_out` = tangent of arc B at its start point (= point B)

The miter intersection algorithm receives these tangents exactly as it
receives direction vectors from lines.  No new algorithm is needed — only
the tangent direction extraction is new.

**Continuity note:**  Fusion 360 produces G1G2/G3 contours where arc-to-arc
transitions are already tangent-continuous.  The intersection algorithm will
find that `denom ≈ 0` (nearly parallel tangents) and will apply the existing
near-parallel fallback, which produces a clean perpendicular offset — the
correct result for a smooth curve-to-curve blend.

### 4.5 Summary Transition Table

| A type  | B (junction) | C type  | Algorithm applied |
|---|---|---|---|
| Line    | Corner       | Line    | Phase 3 miter (unchanged) |
| Line    | Arc entry    | Arc     | Miter using line dir + arc entry tangent |
| Arc     | Arc exit     | Line    | Miter using arc exit tangent + line dir |
| Arc     | Arc junction | Arc     | Miter using two arc tangents |
| Any     | Onset        | Any     | Degenerate: perpendicular to outgoing direction |
| Any     | Terminal     | Any     | Degenerate: perpendicular to incoming direction |

### 4.6 Computation Cases and Dispatch Efficiency

Yes, these transitions should be treated as explicit computation cases with
clear priority, similar to existing priming and flush exceptions.

Proposed evaluation order in `compute_and_output()`:

1. Pipeline state exceptions first:
   - not enough lookahead
   - comp ring full
   - flushing terminal case
2. Geometry dispatch second:
   - classify A/B/C move types into one of 6 transition classes
   - run dedicated compute function for that class

A compact switch-style dispatch is recommended:

```cpp
enum class TransitionCase : uint8_t {
   Onset,
   Terminal,
   LineLine,
   LineArc,
   ArcLine,
   ArcArc
};

switch (transition_case) {
   case TransitionCase::Onset:    return compute_onset(...);
   case TransitionCase::Terminal: return compute_terminal(...);
   case TransitionCase::LineLine: return compute_line_line(...);
   case TransitionCase::LineArc:  return compute_line_arc(...);
   case TransitionCase::ArcLine:  return compute_arc_line(...);
   case TransitionCase::ArcArc:   return compute_arc_arc(...);
}
```

This keeps branching predictable, keeps each solver small, and makes corner
case behavior auditable in tests.

---

## 5. Hard Error Policy for Unsupported Arc Dialects

### 5.1 R-Format Arcs Under G41/G42

R-format arcs (`G2 X10 Y0 R5`) are not supported in Phase 1.  If the
firmware encounters an R word in a G2 or G3 command while G41 or G42 is
active, it must:

1. Immediately stop the run (issue a HALT).
2. Print a clear diagnostic to all streams:
```
Error: R-format arc (G2/G3 R...) is not supported with G41/G42 compensation.
Use I/J offsets instead.  Compensation disabled.
```
3. Turn compensation off (call `set_compensation(NONE, 0)`) so the machine
   is in a known state.

This is a deliberate hard error — the user needs to know their CAM post
processor must output I/J arcs, not R arcs, before compensated arc support
can be used.

### 5.2 G18 / G19 Plane Under G41/G42

Compensation is restricted to G17 (XY plane) in Phase 1.  If G18 or G19
is the active plane when G41 or G42 is issued, the same hard-error and halt
sequence applies:

```
Error: G41/G42 compensation requires G17 (XY plane).
G18/G19 compensation is not yet supported.  Compensation disabled.
```

No code will be silently skipped.  The operator receives an unambiguous
failure that explains what plane is required.

The plane check happens in the G41/G42 handler in `Robot.cpp`, before
`set_compensation()` is called.  `plane_axis_0`, `plane_axis_1`, and
`plane_axis_2` are already available at that point via the existing
`select_plane()` machinery.

---

## 6. Segment-Skip Phase (Separate, High Priority)

### 6.1 The Problem

Standard cutter compensation rules (FANUC, RS274, ISO 6983) state that
when any move segment in the compensated path is too short for the tool
radius to clear — meaning the perpendicular offset of the move places the
tool endpoint behind its start — the segment must be dropped and the adjacent
segments' intersection must be found directly:

```
A → B → C → D

If the compensated form of B→C is too short to execute (or collapses
entirely), drop B→C and compute the intersection of offset(A→B) and
offset(C→D) directly.
```

This can cascade: if multiple consecutive segments are too short, they are
all dropped until a valid intersection is found.

### 6.2 Why This Needs Its Own Phase

This problem is non-trivial to implement correctly in the current dual-ring
architecture because:

1. **Buffer depth**: Detecting a too-short segment requires looking ahead to
   the segment *after* the problem segment.  With BUFFER_SIZE = 3 this is
   at the very limit of available lookahead.  If a cascade of multiple short
   segments occurs, more lookahead slots may be required.
2. **Position state**: When a segment is dropped, the modal position tracking
   must still advance to the endpoint of the dropped segment so subsequent
   segments compute from the correct location.
3. **Testing surface**: The failure mode (tool gouging due to missed
   segment-skip) is geometric and requires a dedicated test suite with
   precisely measured expected outputs before implementation can be accepted.
4. **Interaction with arc support**: An arc that subtends too small an angle
   at the compensated radius is a related but distinct variant of this
   problem.  Both should be designed together.

**Recommendation:** Implement arc support (Phases 1–2 described above) first
and validate it completely.  Then dedicate a subsequent phase specifically to
segment-skip, sized independently after arc support is stable.

### 6.3 Standards Basis and Sources

The segment-drop and adjacent-intersection behavior is grounded in canonical
tool-radius compensation behavior documented by controller standards and
implementations.

Primary references to include in this project documentation:

1. NIST RS274/NGC Interpreter (Kramer, Proctor, Messina): cutter radius
   compensation semantics and geometric handling expectations.
   - Public record: https://www.nist.gov/publications/nist-rs274ngc-interpreter-version-3
   - Canonical text title: The NIST RS274NGC Interpreter - Version 3
2. LinuxCNC Documentation, Cutter Compensation chapter: practical behavior,
   short-segment constraints, and controller handling patterns derived from
   RS274NGC.
   - Documentation: https://linuxcnc.org/docs/html/gcode/tool-compensation.html
3. FANUC Operator Manual (series-specific): tool nose/radius compensation
   constraints, incompatibilities, and alarm behavior for non-resolvable
   geometry.
   - Example manual family: FANUC Series 0i/30i/31i operator manuals
   - Project action: pin exact manual revision used by the Carvera controller
     profile during implementation review
4. Smid, Peter. CNC Programming Handbook: geometric interpretation of offset
   path construction and transition handling.
   - Reference text: CNC Programming Handbook, 3rd Edition, ISBN 9780831133474

These references should be version-pinned in the final implementation
milestone document.

### 6.4 Solution Preview for Segment-Drop (A->B, B->C, C->D)

The idea of dropping directly in compute is valid and likely required.
Below are candidate strategies.

Option A: In-compute single-drop with bounded cascade

- Detect invalid compensated segment for B->C.
- Mark B->C dropped, do not emit output for this step.
- Recompute intersection using A->B and C->D.
- Allow repeated drops with a strict per-output cap (for example
  `MAX_DROPS_PER_OUTPUT = 4`) to prevent runaway loops.

Pros: minimal architecture change.
Risk: edge-case starvation if many tiny segments arrive in sequence.

Option B: Deferred-drop queue

- Tag short segments during intake.
- Resolve drops in a dedicated pass before emission.
- Emit only stable intersections to comp ring.

Pros: deterministic and easier to debug.
Risk: higher code complexity and state bookkeeping.

Option C: Temporary lookahead expansion during drop resolution

- Keep BUFFER_SIZE=3 for steady state.
- When a short segment is detected, borrow temporary lookahead slots from a
  small side buffer used only by the drop resolver.

Pros: no global buffer-size increase.
Risk: most complex implementation.

Option D: Increase fixed ring-buffer size for typical short-segment cases

- Increase the preprocessor lookahead depth from 3 to a larger fixed value
   sized for realistic CAM output patterns.
- Use the extra native lookahead to resolve most drop/intersection cases
   directly inside the main compute pass without special temporary storage.

Pros: simple mental model, simple control flow, and likely resolves most
typical Fusion-generated short-segment cases.
Risk: permanent memory increase across all runs, including jobs that never
need the extra lookahead.

Option E: Dynamically grow and shrink lookahead depth with guardrails

- Start from the normal steady-state depth.
- Expand buffer capacity when drop resolution requires more lookahead.
- Impose a strict maximum capacity to prevent unbounded growth.
- After a successful intersection is emitted without needing a drop, shrink
   capacity back down by one slot at a time until baseline depth is restored.

Pros: memory use tracks actual geometric difficulty instead of worst-case
reservation.
Risk: highest state-management complexity; dynamic resizing must not disturb
ring indices, ownership, or deterministic timing.

Recommended starting point for the segment-drop phase:

- Option A with strict drop cap + explicit alarm when cap is exceeded.
- Alarm message example:
  `Error: compensation drop cascade exceeded limit; geometry unresolved.`

This gives controlled behavior early and prevents silent runaway conditions.

---

## 7. Existing Tolerance Anchors

Rather than introducing new knobs, the proposed implementation reuses the
existing arc tolerance variables already in the firmware:

| Variable | Config key | Default | Usage in arc compensation |
|---|---|---|---|
| `mm_max_arc_error` | `mm_max_arc_error` | 0.002 mm | Defines acceptable chord error; used in segment count calculation in `append_arc()` — serves as the geometric tolerance reference for acceptance testing |
| `mm_per_arc_segment` | `mm_per_arc_segment` | 0.0 (off) | Fixed segment length override |
| `arc_correction` | `arc_correction` | 5 | Periodic exact-correction interval in segment loop to prevent floating-point drift accumulation |

The default `mm_max_arc_error` of 0.002 mm (2 µm) is already tighter than
the tolerance achievable on the Carvera's mechanical system — it is the
correct acceptance threshold for arc compensation correctness tests.

---

## 8. Changes to COMP_LB Report

One field will be added to the per-run load-balance report at G40 time:

```
arc_in=N
```

This counts how many G2/G3 commands entered the compensator during the run.
It slots into the existing COMP_LB line with zero additional per-move cost.
Example output:

```
COMP_LB: in=160 gen=160 srv=160 arc_in=40 compute=159 ...
```

This makes it immediately visible in Kivy logs whether arc moves were seen
and compensated, without requiring trace output to be enabled.

---

## 9. Files That Will Change

| File | Nature of change |
|---|---|
| `CompensationPreprocessor.h` | Extend `UncompPoint` struct with motion_mode, ij[2], feedrate; add `arc_in` to `LoadBalanceMetrics`; declare `compute_arc_compensation()` |
| `CompensationPreprocessor.cpp` | Extend `buffer_gcode()` to capture I/J and motion mode; extend `compute_and_output()` to branch on motion mode; add `compute_arc_compensation()`; update output snprintf to emit G2/G3 when appropriate; update `print_load_balance_report()` with arc_in field; update `reset_load_balance_metrics()` |
| `Robot.cpp` | Add plane check in G41/G42 handler; add R-format detection and hard error; pass motion mode and I/J into `buffer_gcode()` |

No other files are changed.  `append_arc()` is not touched — it continues
to operate on the compensated G2/G3 output that the preprocessor emits.

---

## 10. Test Strategy

### 10.1 Phase 1 Test Programs Required

A dedicated test folder is proposed:
```
tests/TEST_basic_cutter_comp/TEST_arc_cutter_comp/
```

**Program 1 — Single CW arc, G41**
```gcode
G41 D3.175
G2 X10 Y0 I5 J0 F1000
G40
```
Expected: compensated arc radius = 5 + 1.5875 = 6.5875 mm

**Program 2 — Single CCW arc, G42**
```gcode
G42 D3.175
G3 X0 Y10 I0 J5 F1000
G40
```
Expected: compensated arc radius = 5 + 1.5875 = 6.5875 mm

**Program 3 — Full circle (arc → same arc, onset = terminal)**
```gcode
G41 D3.175
G2 X0 Y0 I10 J0 F1000
G40
```
Expected: circle closes, no gap between onset and terminal points.

**Program 4 — Line → Arc → Line**
```gcode
G41 D3.175
G1 X10 Y0 F1000
G2 X20 Y10 I0 J10
G1 X20 Y20
G40
```
Expected: smooth compensated tangent-continuous transitions at both
line-to-arc and arc-to-line junctions.

**Program 5 — Arc → Arc junction**
```gcode
G41 D3.175
G2 X10 Y0 I5 J0 F1000
G3 X20 Y10 I0 J10
G40
```
Expected: junction at (10,0) is tangent-continuous; no kink or offset step.

**Program 6 — Helical arc (Z)**
```gcode
G41 D3.175
G2 X10 Y0 Z5 I5 J0 F1000
G40
```
Expected: XY compensation correct; Z = 5.0 at endpoint.

**Program 7 — R-format hard error**
```gcode
G41 D3.175
G2 X10 Y0 R5 F1000
G40
```
Expected: hard error + halt before any motion; clear message in Kivy log.

**Program 8 — Stress: compensated arc at 6000 mm/min**
Fusion-style arc contour at max feed.
Expected: COMP_LB shows miss=0.0%, empty_srv=0, prod_vs_pull ≥ 99.5%.

### 10.2 Acceptance Criteria

| Criterion | Pass threshold |
|---|---|
| Geometric correctness | Compensated arc radius error ≤ mm_max_arc_error (0.002 mm default) |
| Transition continuity | No offset step or kink visible at line/arc junctions |
| Helical Z fidelity | Z at arc endpoint matches programmed Z exactly |
| Hard error behavior | R-format and non-G17 arcs halt with clear message; no silent pass-through |
| Load balance | miss=0.0%, empty_srv=0, prod_vs_pull ≥ 99.5% at 6000 mm/min |
| No regression | Existing segmented-line stress test COMP_LB unchanged from baseline |
| Full circle closure | Arc-to-self endpoint error ≤ mm_max_arc_error |
| No halt/freeze | Run completes without watchdog or conveyor stall |

### 10.3 Test File Layout and Headers

Each test in `tests/TEST_basic_cutter_comp/TEST_arc_cutter_comp/` should have
a header block describing intent and expected behavior. Example header:

```gcode
(TEST_ID: ARC_01_SINGLE_CW_G41)
(INTENT: Validate native G2 compensation in G17 with I/J format)
(EXPECT: Radius offset correct; no hard error; COMP_LB miss=0)
(NOTES: Use 6000 mm/min stress rerun after baseline pass)
```

### 10.4 Deep Debug Build Profile for Arc Bring-Up

For initial arc bring-up only, turn deep compensation trace on in a dedicated
test build profile:

- `CUTTER_COMPENSATION_TRACE_ENABLED = 1`

Guideline:

1. Use trace-enabled firmware only for short deterministic test files.
2. Disable trace for throughput validation runs (to avoid I/O bottleneck).
3. Keep COMP_LB reporting enabled in both profiles.

Recommended artifact naming convention:

- `firmware_vX.Y_ARC_PHASE1_TRACE_ON_YYYYMMDD.bin`
- `firmware_vX.Y_ARC_PHASE1_TRACE_OFF_YYYYMMDD.bin`

---

## 11. What Is Not Changing

To be explicit about scope boundaries:

- `append_arc()` and `compute_arc()` in Robot.cpp are not modified.
- Buffer size (BUFFER_SIZE = 3) is not changed in this phase.
- G40 flush behavior, drain loop, and load-balance report point are not changed.
- Phase-locked dual-ring clocking contract is not changed.
- G18/G19 compensation is not added (hard error if attempted).
- Segment-skip (too-small move merge) is not implemented here.

---

## Sign-Off

| Role | Status |
|---|---|
| Design | ⏳ Awaiting review |
| Implementation | 🔒 Blocked until design approved |
| Test programs | 🔒 Blocked until design approved |

---

*This document describes the proposed design only.  No implementation work
will begin until the design has been reviewed and approved by the project
owner.*
