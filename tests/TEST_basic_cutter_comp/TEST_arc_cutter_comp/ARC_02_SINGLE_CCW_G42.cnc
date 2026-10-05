(TEST_ID: ARC_02_SINGLE_CCW_G42)
(INTENT: Validate native G3 compensation in G17 with I/J format)
(EXPECT: Radius offset correct; no hard error; COMP_LB miss=0)
(NOTES: Baseline CCW arc test)
(NOTES: Programmed lead-in from X-3 approaches arc start along arc start tangent (+X))
(NOTES: Arc start tangent=(+1,0); G42 right offset=(-Y); comp radius=5+1.587=6.587)
(NOTES: Lead-out departs arc end along arc end tangent (-X))
G90 G94
G17
G21
G54
G0 X-3 Y0 Z5
G1 Z-1 F100
G42 D3.175
G1 X0 Y0 F300
G3 X0 Y10 I0 J5 F1000
G1 X-3 Y10
G40
G0 Z10
M30
