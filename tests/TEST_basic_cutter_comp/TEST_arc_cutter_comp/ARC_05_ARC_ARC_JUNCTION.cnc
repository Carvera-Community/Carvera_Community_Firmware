(TEST_ID: ARC_05_ARC_ARC_JUNCTION)
(INTENT: Validate arc-to-arc tangent continuity)
(EXPECT: No offset step at arc junction)
(NOTES: Mixed CW/CCW transition)
(NOTES: Arc1 G2 from (0,0) ctr=(5,0); start tangent=(0,+1); lead-in from Y-3)
(NOTES: Arc2 G3 to (20,10) ctr=(10,10); end tangent=(0,+1); lead-out to Y13)
(NOTES: Arc1/arc2 junction is a 90-deg corner; eff_radius fix eliminates trailing line)
G90 G94
G17
G21
G54
G0 X0 Y-3 Z5
G1 Z-1 F100
G41 D3.175
G1 X0 Y0 F300
G2 X10 Y0 I5 J0 F1000
G3 X20 Y10 I0 J10
G1 X20 Y13
G40
G0 Z10
M30
