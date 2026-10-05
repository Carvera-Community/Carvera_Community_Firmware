(TEST_ID: GATE_03_BLOCKED_M_CODES)
(INTENT: Verify blocked M-code is rejected while compensation is active)
(EXPECT: Halt on M3 with message "M3 is not supported while G41/G42 compensation is active")

G90 G94
G17
G21
G54

M118 GATE_03 start
G0 X0 Y0 Z5
G1 Z-1 F300
G41 D3.175

M118 GATE_03 about to issue blocked M3
M3 S8000

M118 GATE_03 ERROR expected halt before this line
G40
M30
