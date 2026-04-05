# Cutter Compensation - Current Quick Reference

This file tracks the current, tested state of cutter compensation behavior in the Carvera Community Firmware.

For detailed implementation history and architecture notes, see:
- `cutter_comp_documentation/CUTTER_COMPENSATION_IMPLEMENTATION.md`

## Current Firmware Status (April 2026)

- Active branch behavior supports G41/G42 compensation in normal G17 XY workflows.
- ARC test suite (`ARC_01` through `ARC_06`) has passed in machine validation.
- Representative Fusion-style load test (`cutter comp v6 proper lead ins finishing pass.cnc`) has passed with stable compensation pipeline metrics.
- Arc endpoint radius handling uses geometric offset radius (`uncomp_radius +/- tool_radius`) to prevent endpoint snap artifacts caused by start-point radius drift.
- Terminal arc compensation uses incoming endpoint tangent for stable final output.

## Required Programming Pattern

- Use explicit, programmed lead-in and lead-out geometry in CAM output.
- Do not rely on compensation activation directly into an arc as the first compensated move.
- Fusion 360-posted lead-in/out paths are the intended workflow.

## Supported and Risk Areas

- Supported: G17 XY lines and I/J arcs under G41/G42.
- Not supported: R-format arcs with active compensation.
- Risk area: mixed-plane moves (for example G19 while compensation remains active) should be validated on-machine for each program.

## Validation Checklist

For each candidate program, verify in Kivy log:

1. Compensation activates and deactivates cleanly (`G41/G42` then `G40`).
2. No firmware hard fault markers (`ALARM`, `hard error`, `abort`).
3. Healthy load-balance report (`COMP_LB miss=0.0%`, `prod_vs_pull` near 100%).
4. Expected compensated geometry at critical transitions (lead-in, arc entry/exit, terminal move).

## Notes

- Occasional `list index out of range` messages observed near `G28` in Kivy appear host-side and are not currently linked to compensation pipeline failure.
