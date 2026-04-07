# 3D Toolsetter → Cutter Compensation v2: Merge Reference

**Date started:** April 5, 2026  
**Branches:** `3dtoolsetter` → `cutter-compensation-v2`  
**Divergence point from master:** `0646581`

---

## Objective

Bring the 3D toolsetter hardware motion sequences, spindle direction control (M4), and tool diameter storage from the `3dtoolsetter` branch into `cutter-compensation-v2` so that measured tool diameter can be captured during the ATC calibration cycle and applied to G41/G42 D values in real time.

---

## Branch State at Merge Start

| Branch | Tip commit | Notes |
|--------|-----------|-------|
| `cutter-compensation-v2` | `40bbf89` | Arc compensation validated, all ARC_01–06 pass, Fusion v6 load test pass (`in=68, miss=0.0%`) |
| `3dtoolsetter` (local) | `4454d2e` | Contains full toolsetter motion + M4 prototype; cutter comp work also diverged here |
| `origin/3dtoolsetter` | `6c46b5b` | Remote is 4 commits behind local tip |

### 3dtoolsetter commit history (newest first)
```
4454d2e  code review
ecb6b42  debugging arc compensation to straight transition
6cc67c2  Single-axis moves causing erratic compensation behavior
6c46b5b  (origin) Fix cutter compensation crash (heap allocation in G40 flush)
3734449  Working out preprocessor sequencing. NOT SAFE YET
c6b5b83  Added midterm documentation
85b32fc  Added more debug output at key points
d26a0c7  First testable build with new preprocessor strategy (unstable)
e208190  Total refactor - all compensation now in preprocessor
1ffd7ec  STILL DEBUGGING - NOT SAFE
a589703  Fixing memory leak after G41 D6
50da5e3  Added test for phase 1
8a64fc0  Added cutter compensation states, fields and helper methods
0e52c10  Outline of strategy for implementing cutter comp
2b7bea8  Added M4 prototype without EXT port control. Not ready for testing yet.
34ce471  Crude TDO cycle test. WARNING: EXT port inverted logic on BLD-300
a2338fb  Tested G38.6 in X, Y, and Z
```

---

## Scope: Toolsetter-Specific Changes Only

The full `3dtoolsetter` branch contains a large volume of cutter compensation work that is superseded by `cutter-compensation-v2`. The target of this merge is **toolsetter hardware motion only** — the 7-file, 147-line delta between master divergence point `0646581` and the `3dtoolsetter` tip, restricted to toolsetter-relevant files.

### Files to Incorporate

| File | Change Summary | Status |
|------|---------------|--------|
| `src/libs/Kernel.h` | Add `float TOOL_DIA` to `EEPROM_data` struct | ⬜ Pending |
| `src/libs/Kernel.cpp` | Add `\|C:` comp status to query string; fix missing `return true` in `process_line()` | ⬜ Pending |
| `src/modules/tools/spindle/SpindleControl.h` | Add `bool output_inverted{false}` and `bool spindle_reversed{false}` | ⬜ Pending |
| `src/modules/tools/spindle/SpindleControl.cpp` | Add M4 (CCW) handler block | ⬜ Pending |
| `src/modules/tools/atc/ATCHandler.cpp` | Add diameter calibration motion sequence after `M493.1` in `fill_cali_scripts()` | ⬜ Pending |
| `src/modules/tools/zprobe/ZProbe.h` | `calibrate_Z()` return type: `void` → `bool` | ⬜ Pending |
| `src/modules/tools/zprobe/ZProbe.cpp` | `calibrate_Z()` accept X/Y/Z, return bool, use `CALIBRATE_FAIL` halt | ⬜ Pending |

### Already Present on Current Branch (No Action Needed)
- `read_calibrate()` ISR: X/Y/Z axis movement detection — already in place
- G38.6 Z probing in `fill_cali_scripts()` — already used for TLO measurement

---

## Diameter Calibration Motion Sequence (from `fill_cali_scripts()`)

After `M493.1` (save new TLO), the following sequence measures tool diameter against the toolsetter:

```
G91 G0 Z2          ; lift above TLO contact plane
G91 G0 X-10.5      ; step over to clear full tool diameter
G91 G1 Z-3.5 F100  ; descend to tool cutting-edge center height
M4 S2000           ; CCW spindle (see EXT port note below)
G38.6 X3.5 F30     ; probe +X until cutting edge touches toolsetter
G91 G0 X-0.5       ; retract from sensor
M5                 ; spindle off
G53 G0 Z<safe_z> F1000.0  ; retract to safe height
```

A slow second-pass probe (commented out in the commit) is preserved as comments for future use.

---

## Known Caveats and TODOs

### EXT Port / M4 Direction (Hardware-Unvalidated)
- The BLD-300 ESC uses **inverted logic** on the EXT port for direction: `Low = CCW, High/NC = CW`
- Commit `2b7bea8` notes: *"Not ready for testing yet"* — EXT port control was explicitly excluded
- The M4 handler uses `output_inverted = !output_inverted` as a prototype; this approach inverts the
  existing PWM output rather than driving a dedicated direction pin — it is not the correct final solution

**Correct approach — dedicated direction pin with config-level inversion:**
- Add a `spindle.dir_pin` config entry in `config.default` / `config2.default` pointing to the EXT port GPIO
- Appending `!` to the pin string (e.g. `1.18!`) sets `is_inverting()` at load time — this is the same
  mechanism already used by `PWMSpindleControl` and `AnalogSpindleControl` for `output_inverted`
- Read the direction pin in `SpindleControl::on_module_loaded()` and store as a `Pin` member (similar to
  how `ModbusSpindleControl` reads `dir_pin`)
- M3 handler: assert direction pin LOW (CW) before calling `turn_on()`
- M4 handler: assert direction pin HIGH (CCW) before calling `turn_on()`; pin configured with `!` in
  config will invert this automatically, matching the BLD-300 EXT port polarity without needing runtime
  inversion in the M3/M4 command logic itself
- **Do not enable hardware direction switching until EXT port wiring and pin assignment are confirmed**

### M493.2 (Store Measured Diameter)
- Placeholder `M493.2` command for saving measured diameter is commented out in the commit
- Needs to be implemented to write the measured X position back to `TOOL_DIA` in `EEPROM_data`

### M491 Tool Sensor Location is Coupled to Tool Offset Chain
- Current behavior: M491 tool-sensor contact motions are derived through an offset chain that starts from tool 1 position
- This makes sensor setup fragile and harder to reason about, because sensor location is indirect rather than explicit
- Target behavior: define tool sensor location directly in absolute machine coordinates (G53 frame)

**Compatibility target:** PR #171 "Configurable Tool Setter Position and one-off calibration offset"
- Keep naming and behavior compatible with PR #171 so existing extended-ATC mods continue to work
- Prefer these config keys for absolute MCS toolsetter position:
  - `coordinate.probe_mcs_x`
  - `coordinate.probe_mcs_y`
  - `coordinate.probe_mcs_z` (reserved for upstream compatibility; not used in this phase)
- Keep one-off per-command offsets on `M491.1` for X/Y in this phase
- Keep/report behavior aligned with `M493.4` status output for effective toolsetter position and active one-off offsets

**Upstream evolution lock:** PR #179 and PR #186
- Follow the same naming family and behavior direction used in upstream community development
- Keep compatibility with custom ATC ecosystems that depend on:
  - `coordinate.probe_mcs_x/y/z`
  - MCS-oriented tool slot thinking (M889 visibility and M890/M891 style workflows)
- Note: PR #186 introduces SD-backed custom slot storage (`/sd/custom_tool_slots.txt`) to reduce memory pressure
  from config line overhead; this is relevant to future persistence planning

**Migration direction (for merge implementation):**
- Load `coordinate.probe_mcs_x/y` from config in `ATCHandler::on_config_reload()`
- Update M491/ATC calibration path to use those absolute values directly when generating contact moves
- Keep backward compatibility by preserving legacy anchor/offset-chain fallback when probe MCS fields are unset
- Keep Z out of configuration architecture for this effort; Z remains dynamically resolved by the existing calibration
  flow each pickup cycle (M493.4/TLO path)
- Add/report lines on separate lines:
  - configured toolsetter coordinates (X/Y only)
  - resolved effective toolsetter coordinates used for the move (X/Y)
  - resolved effective probe target Z (clearly labeled as dynamic/fallback, not configured)
- Validate by moving to sensor from multiple active tools and confirming identical machine target coordinates in each case

**Axis policy decision (current):**
- Primary requirement is decoupling X and Y from anchor/tool-offset chain
- Z is not configured or labeled as configured in this phase
- Z continues to follow existing tool-height sensing behavior and is resolved each pickup/calibration cycle
- If a future tool table emerges, Z persistence remains optional because repeated measurement may still be preferred due to
  clamp repeatability limits

### Decision Brief: X/Y-only Config Now vs X/Y/Z Config Now

**Option A: X/Y-only configuration now (recommended for this merge)**
- Pros:
  - Matches current machine reality: Z is re-measured during pickup/calibration and is not trusted as a static config value
  - Lower regression risk in ATC flows that depend on dynamic Z clamp behavior
  - Keeps the decoupling objective focused on removing anchor-chain coupling for planar sensor location
  - Easier compatibility with existing mods that mostly need planar relocation of sensor/tool arrays
- Cons:
  - Less flexibility for edge cases where a fixed Z offset is intentionally desired
  - May require future extension once upstream introduces a broader tool metadata/table model

**Option B: X/Y/Z configuration now**
- Pros:
  - Maximum configurability immediately
  - Aligns with PR #171 key set as originally introduced
  - Could reduce custom patching for users with atypical hardware geometry
- Cons:
  - Conflicts with current operational pattern where Z is re-resolved each pickup
  - Higher risk of stale/wrong Z config causing probing issues or unsafe moves
  - Encourages treating Z as static when clamp variation makes dynamic measurement safer

**Recommendation for current branch:**
- Implement Option A (X/Y-only configured, Z dynamic each cycle)
- Keep `coordinate.probe_mcs_z` as a reserved compatibility key name only (ignored in this phase), so future alignment with
  upstream evolution remains straightforward

### Real-Time D-Value Update (Phase 2 — Not Yet Started)
- The end goal is: measured diameter → `THEKERNEL->eeprom_data->TOOL_DIA` → G41/G42 radius source
- `Robot.cpp` G41/G42 handler currently reads only from the `D` word in G-code
- After this merge, phase 2 work will wire `TOOL_DIA` as a fallback when D is 0 or absent

### Diameter Persistence Proposal (Aligned with PR #179/#186 Direction)
- Current proposal: keep measured diameter persistence temporary (`TOOL_DIA` runtime/session level) until a true tool-table
  pattern stabilizes in upstream community firmware
- Observed direction from PR #179/#186: strong movement toward configurable tool-slot infrastructure, but not yet a full
  per-tool diameter table in the same persistence model
- Trigger to invest in persistent diameter storage:
  - Upstream introduces or converges on per-tool persistent metadata beyond XYZ slot location (for example, an extensible
    tool record or tool table file format)
- If/when that trigger appears, proposed compatibility path:
  - Extend tool slot persistence model with diameter field (or paired metadata file) without breaking existing slot files
  - Add migration/compat parser that treats missing diameter as unknown and preserves legacy behavior
  - Bind G41/G42 fallback to active tool's persisted diameter when available, else runtime `TOOL_DIA`

---

## Progress

- [ ] Apply 7-file toolsetter changes to working tree
- [ ] Verify clean build
- [ ] Implement `M493.2` to store measured diameter into `TOOL_DIA`
- [ ] Decouple M491 tool sensor location from tool-1 offset chain and use G53 machine coordinates (X/Y)
- [ ] Keep decoupling implementation compatible with PR #171/#179/#186 naming and behavior direction
- [ ] Update M493.4-style reporting on separate lines for configured X/Y and resolved effective X/Y/Z (with Z marked dynamic)
- [ ] Update G41/G42 handler in `Robot.cpp` to use `TOOL_DIA` when D is absent or zero
- [ ] Machine-validate diameter measurement on physical hardware
- [ ] Machine-validate real-time G41/G42 with measured diameter
