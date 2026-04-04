(TEST_ID: ARC_03_FULL_CIRCLE_CLOSURE)
(INTENT: Validate full-circle closure under compensation)
(EXPECT: No endpoint gap larger than tolerance)
(NOTES: Full-circle edge case)
G90 G94
G17
G21
G54
G0 X10 Y0 Z5
G1 Z-1 F600
G41 D3.175
G2 X10 Y0 I-10 J0 F1000
G40
G0 Z10
M30
