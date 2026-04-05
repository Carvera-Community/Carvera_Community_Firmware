(TEST_ID: ARC_01_SINGLE_CW_G41)
(INTENT: Validate native G2 compensation in G17 with I/J format)
(EXPECT: Radius offset correct; no hard error; COMP_LB miss=0)
(NOTES: Baseline CW arc test)
(NOTES: Programmed lead-in from Y-3 approaches arc start along arc start tangent (+Y))
(NOTES: Arc start tangent=(0,+1); G41 left offset=(-X); comp radius=5+1.587=6.587)
(NOTES: Lead-out departs arc end along arc end tangent (-Y))
G90 G94
G17
G21
G54
G0 X0 Y-3 Z5
G1 Z-1 F100
G41 D3.175
G1 X0 Y0 F300
G2 X10 Y0 I5 J0 F1000
G1 X10 Y-3
G40
G0 Z10
M30
