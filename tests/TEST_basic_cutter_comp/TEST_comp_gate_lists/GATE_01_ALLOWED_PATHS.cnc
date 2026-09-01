(TEST_ID: GATE_01_ALLOWED_PATHS)
(INTENT: Verify non-blocked non-motion commands pass while G41/G42 is active)
(EXPECT: Program reaches M30 without compensation gate halt)
(EXPECT: G18/G19 freeze-resume path executes without hard stop)

G90 G94
G17
G21
G54

M118 GATE_01 start
G0 X0 Y0 Z5
G1 Z-1 F300
G41 D3.175

M118 GATE_01 comp on
G4 P0
M114
M400
G54
G90

G1 X10 Y0 F800
G18
G1 X20 Z-1 F500
G19
G1 Y10 Z-1 F500
G17
G1 X0 Y0 F800

G40
M118 GATE_01 done
G0 Z5
M30
