(TEST_ID: ARC_03_FULL_CIRCLE_CLOSURE)
(INTENT: Validate full-circle closure under compensation)
(EXPECT: No endpoint gap larger than tolerance)
(NOTES: Full-circle edge case)
(NOTES: Arc starts at (10,0), center (0,0). Start tangent=(0,-1) (CW, moving south))
(NOTES: Lead-in approaches from Y+3 along -Y; G41 left offset=(+X); comp r=11.587)
(NOTES: Lead-out continues in -Y direction after full-circle return to (10,0))
G90 G94
G17
G21
G54
G0 X10 Y3 Z5
G1 Z-1 F100
G41 D3.175
G1 X10 Y0 F300
G2 X10 Y0 I-10 J0 F1000
G1 X10 Y-3
G40
G0 Z10
M30
