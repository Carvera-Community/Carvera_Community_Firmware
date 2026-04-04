(TEST_ID: ARC_08_STRESS_6000)
(INTENT: Throughput stress with compensated arcs at max feed)
(EXPECT: miss=0, empty_srv=0, prod_vs_pull>=99.5%, no visible stutter)
(NOTES: Keep trace off for final throughput validation)
G90 G94
G17
G21
G54
G0 X-10 Y0 Z5
G1 Z-1 F6000
G41 D3.175
G2 X10 Y0 I10 J0 F6000
G3 X-10 Y0 I-10 J0
G2 X10 Y0 I10 J0
G3 X-10 Y0 I-10 J0
G40
G0 Z10
M30
