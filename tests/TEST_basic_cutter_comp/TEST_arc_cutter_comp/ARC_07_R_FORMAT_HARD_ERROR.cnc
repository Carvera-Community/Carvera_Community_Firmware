(TEST_ID: ARC_07_R_FORMAT_HARD_ERROR)
(INTENT: Verify unsupported R-format hard error with G41/G42)
(EXPECT: Hard error and halt before compensated motion)
(NOTES: Negative compatibility test)
G90 G94
G17
G21
G54
G0 X0 Y0 Z5
G1 Z-1 F600
G41 D3.175
G2 X10 Y0 R5 F1000
G40
G0 Z10
M30
