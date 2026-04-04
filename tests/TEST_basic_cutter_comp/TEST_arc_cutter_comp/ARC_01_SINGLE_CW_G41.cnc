(TEST_ID: ARC_01_SINGLE_CW_G41)
(INTENT: Validate native G2 compensation in G17 with I/J format)
(EXPECT: Radius offset correct; no hard error; COMP_LB miss=0)
(NOTES: Baseline CW arc test)
G90 G94
G17
G21
G54
G0 X0 Y0 Z5
G1 Z-1 F600
G41 D3.175
G2 X10 Y0 I5 J0 F1000
G40
G0 Z10
M30
