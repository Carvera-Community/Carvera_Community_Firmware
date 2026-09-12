%
(Phase 1 Test: Single Arc with Robust Setup)
(Always moves to start position first, then enables compensation)

G21 (metric)
G90 (absolute)
G17 (XY plane)
F1000 (feed rate)

(Cancel any active compensation and move to start position)
G40
G0 X0 Y0 Z0

(Enable left compensation - creates approach move from current position)
G41 D6

(Arc move - semicircle from X0 to X20)
G2 X20 Y0 I10 J0

(Line move to complete test)
G1 X30 Y0

(Disable compensation and end)
G40
M2
%
