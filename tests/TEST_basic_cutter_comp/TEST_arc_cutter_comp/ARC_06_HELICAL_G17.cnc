(TEST_ID: ARC_06_HELICAL_G17)
(INTENT: Validate helical arc support with Z change)
(EXPECT: Correct XY compensation and correct terminal Z)
(NOTES: Helical arc in G17)
(NOTES: Same XY geometry as ARC_01; start tangent=(0,+1); lead-in from Y-3 at Z-1)
(NOTES: Arc descends Z-1 to Z-3; lead-out stays at Z-3)
G90 G94
G17
G21
G54
G0 X0 Y-3 Z5
G1 Z-1 F100
G41 D3.175
G1 X0 Y0 F300
G2 X10 Y0 Z-3 I5 J0 F300
G1 X10 Y-3
G40
G0 Z10
M30
