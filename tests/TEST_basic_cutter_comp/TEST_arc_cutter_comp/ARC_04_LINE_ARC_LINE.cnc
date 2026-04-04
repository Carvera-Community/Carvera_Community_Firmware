(TEST_ID: ARC_04_LINE_ARC_LINE)
(INTENT: Validate line to arc to line continuity)
(EXPECT: No visible kink at transition junctions)
(NOTES: Transition test)
G90 G94
G17
G21
G54
G0 X0 Y0 Z5
G1 Z-1 F600
G41 D3.175
G1 X10 Y0 F1000
G2 X20 Y10 I0 J10
G1 X20 Y20
G40
G0 Z10
M30
