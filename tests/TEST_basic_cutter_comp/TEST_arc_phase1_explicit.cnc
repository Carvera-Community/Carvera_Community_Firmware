%
(Phase 1 Test: Arc with Explicit Setup Sequence)
(Forces setup to complete before compensation enabled)

G21 (metric)
G90 (absolute)
G17 (XY plane)

(Step 1: Cancel compensation and wait)
G40
M400 (wait for all moves to complete)

(Step 2: Move to start position and wait)
G0 X0 Y0 Z0
M400

(Step 3: Set feed rate)
F1000

(Step 4: Enable compensation - should create zero-length skip)
G41 D6

(Step 5: Arc move)
G2 X20 Y0 I10 J0

(Step 6: Line move)
G1 X30 Y0

(Step 7: Disable compensation)
G40

(End program)
M2
%
