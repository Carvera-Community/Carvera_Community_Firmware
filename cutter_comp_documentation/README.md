# Cutter Compensation Refactor - Quick Reference

## Folder Contents

### `/algorithms/`
- `core_compensation_math.cpp` - Proven working math algorithms from v14
  - Perpendicular offset for straight lines
  - Corner intersection calculation
  - Arc endpoint offset (tangent-based)
  - Geometry utilities

### `/architecture/`
- `BOLT_ON_ARCHITECTURE_V2.md` - Complete v2.0 design specification
  - Gcode-in/Gcode-out approach
  - API design
  - Integration points
  - Testing strategy

### `/test_files/`
- (To be copied from current branch before checkout)

---

## Key Insights from v1.0

### What Worked Well ✅
1. **Circular buffer pattern** - Efficient, no heap allocation
2. **Lookahead depth of 3** - Perfect for corner detection
3. **Core math algorithms** - Perpendicular offset, corner intersection, arc offset all correct
4. **G40/G41/G42 parsing** - Clean integration in Robot::on_gcode_received()

### What Needs Refactoring ❌
1. **Dual execution paths** - process_move() vs. process_parsed_move()
2. **ParsedMove structure** - Created parallel data stream
3. **Coordinate transform duplication** - 150+ lines copied from process_move()
4. **Manual machine_position tracking** - Error-prone state management
5. **Conditional buffering** - Only buffering when compensation active
6. **No lead-in knowledge** - Can't see next move when G41/G42 activated

---

## v2.0 Core Principles

1. **Single Execution Path**
   - ALL moves go through process_move()
   - Compensation modifies Gcode coordinates
   - No parallel processing

2. **Always-On Buffering**
   - Buffer ALL moves, regardless of compensation state
   - Provides lookahead context for lead-in calculation
   - Stream-based processing - commands flow through sequentially
   - Minimal flush requirements (only for tool changes)

3. **Bolt-On Architecture**
   - Compensation is transparent to Robot
   - Minimal changes to existing code
   - Easy to disable/remove

4. **Memory vs. Maintainability**
   - Accept 2.2KB extra memory for always-on buffer
   - Gain 50% code complexity reduction
   - Simpler logic - no conditional buffering
   - Worth the tradeoff on 64KB RAM system

---

## Implementation Order

1. Create fresh branch from current master
2. Implement Gcode manipulation utilities
3. Implement buffer with Gcode* storage
4. Port proven math algorithms
5. Integrate into Robot::on_gcode_received()
6. Test incrementally
7. Remove old code

---

## Testing Philosophy

**Incremental validation:**
- Test after each component implementation
- Compare behavior with v14 firmware
- Use existing test/*.cnc files
- Debug single execution path (easier!)

**Test progression:**
1. Basic lines (G1 with G41/G42)
2. Corners (rectangles)
3. Arcs (G2/G3)
4. Combined (complex paths)

---

## Decision Rationale

**Why fresh branch?**
- Current branch based on ancient commit (0646581)
- Architecture fundamentally flawed
- Faster to rebuild with correct design
- Preserve lessons learned, not broken code

**Why Gcode modification?**
- Single execution path
- Automatic Robot feature compatibility
- Worth the string reconstruction overhead
- Cleaner architecture for maintainers

**Why now?**
- Hit debugging wall with dual-stream
- Better foundation before more features
- Easier for Makera team to maintain

---

## Success Metrics

**Code Quality:**
- [ ] Net -150 lines of code
- [ ] Single execution path
- [ ] No WCS transform duplication

**Functionality:**
- [ ] Matches v14 behavior on test cases
- [ ] Works through process_move()
- [ ] Soft limits, overrides work automatically

**Maintainability:**
- [ ] Easy to understand data flow
- [ ] Clear component boundaries
- [ ] Documented architecture

---

## Files to Reference During Implementation

From v1.0:
- `CompensationPreprocessor.cpp` lines 290-540 (math algorithms)
- `Robot.cpp` lines 574-640 (G40/G41/G42 handling)
- `Robot.cpp` lines 1220-1260 (current integration)
- All `tests/TEST_*.cnc` files

New in v2.0:
- `architecture/BOLT_ON_ARCHITECTURE_V2.md` (complete design)
- `algorithms/core_compensation_math.cpp` (proven math)

---

**Created:** January 15, 2026  
**Purpose:** Knowledge preservation for cutter compensation refactor  
**Status:** Ready for fresh branch implementation
