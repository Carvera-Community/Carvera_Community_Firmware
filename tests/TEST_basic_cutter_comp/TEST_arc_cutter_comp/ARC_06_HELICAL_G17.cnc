(TEST_ID: ARC_06_HELICAL_G17)
(INTENT: Validate helical arc support with Z change)
(EXPECT: Correct XY compensation and correct terminal Z)
(NOTES: Helical arc in G17)
G90 G94
G17
G21
G54
G0 X0 Y0 Z5
G1 Z-1 F600
G41 D3.175
G2 X10 Y0 Z-3 I5 J0 F1000
G40
G0 Z10
M30
