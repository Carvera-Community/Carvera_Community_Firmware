(TEST_ID: GATE_04_CONDITIONAL_MODE_WCS)
(INTENT: Verify conditional guards only block mode/WCS changes, not no-op restates)
(EXPECT: G90 restate passes while already absolute)
(EXPECT: G54 restate passes while current WCS is G54)
(EXPECT: Halt on G91 change attempt while comp active)

G90 G94
G17
G21
G54

M118 GATE_04 start
G0 X0 Y0 Z5
G1 Z-1 F300
G41 D3.175

M118 GATE_04 no-op state restates
G90
G54

M118 GATE_04 about to issue blocked G91
G91

M118 GATE_04 ERROR expected halt before this line
G40
M30
