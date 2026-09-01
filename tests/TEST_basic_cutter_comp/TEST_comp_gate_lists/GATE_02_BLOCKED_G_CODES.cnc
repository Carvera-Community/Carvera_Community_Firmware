(TEST_ID: GATE_02_BLOCKED_G_CODES)
(INTENT: Verify blocked G-code is rejected while compensation is active)
(EXPECT: Halt on G53 with message "G53 is not supported while G41/G42 compensation is active")

G90 G94
G17
G21
G54

M118 GATE_02 start
G0 X0 Y0 Z5
G1 Z-1 F300
G41 D3.175

M118 GATE_02 about to issue blocked G53
G53 G0 X0

M118 GATE_02 ERROR expected halt before this line
G40
M30
