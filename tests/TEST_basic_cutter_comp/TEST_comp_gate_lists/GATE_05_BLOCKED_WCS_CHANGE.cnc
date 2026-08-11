(TEST_ID: GATE_05_BLOCKED_WCS_CHANGE)
(INTENT: Verify active-WCS change is blocked while compensation is active)
(EXPECT: Halt on G55 with message "G5x work coordinate changes are not supported while G41/G42 compensation is active")

G90 G94
G17
G21
G54

M118 GATE_05 start
G0 X0 Y0 Z5
G1 Z-1 F300
G41 D3.175

M118 GATE_05 about to issue blocked G55
G55

M118 GATE_05 ERROR expected halt before this line
G40
M30
