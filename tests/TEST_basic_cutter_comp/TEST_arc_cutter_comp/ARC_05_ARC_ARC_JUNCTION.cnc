(TEST_ID: ARC_05_ARC_ARC_JUNCTION)
(INTENT: Validate arc-to-arc tangent continuity)
(EXPECT: No offset step at arc junction)
(NOTES: Mixed CW/CCW transition)
G90 G94
G17
G21
G54
G0 X0 Y0 Z5
G1 Z-1 F600
G41 D3.175
G2 X10 Y0 I5 J0 F1000
G3 X20 Y10 I0 J10
G40
G0 Z10
M30
