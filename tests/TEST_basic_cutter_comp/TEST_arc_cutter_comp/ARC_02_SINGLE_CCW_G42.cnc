(TEST_ID: ARC_02_SINGLE_CCW_G42)
(INTENT: Validate native G3 compensation in G17 with I/J format)
(EXPECT: Radius offset correct; no hard error; COMP_LB miss=0)
(NOTES: Baseline CCW arc test)
G90 G94
G17
G21
G54
G0 X0 Y0 Z5
G1 Z-1 F600
G42 D3.175
G3 X0 Y10 I0 J5 F1000
G40
G0 Z10
M30
