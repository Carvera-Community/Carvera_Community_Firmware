(TEST_ID: ARC_04_LINE_ARC_LINE)
(INTENT: Validate line to arc to line continuity)
(EXPECT: No visible kink at transition junctions)
(NOTES: Transition test)
(NOTES: Arc G2 from (10,0) center=(10,10); start tangent=(-1,0) (CW, moving west))
(NOTES: Lead-in from X+13 approaches arc start along -X (tangent); G41 left=(-Y))
(NOTES: Arc end at (20,10), end tangent=(0,-1); lead-out continues south to Y7)
G90 G94
G17
G21
G54
G0 X13 Y0 Z5
G1 Z-1 F100
G41 D3.175
G1 X10 Y0 F300
G2 X20 Y10 I0 J10
G1 X20 Y7
G40
G0 Z10
M30
