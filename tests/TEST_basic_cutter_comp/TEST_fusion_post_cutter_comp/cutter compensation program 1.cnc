(cutter compensation program 1)
(T1  25mm Flute End Mill  D=3.175 CR=0 - ZMIN=-6 - flat end mill)
G90 G94
G17
G21

(2D Contour1)
T1 M6
S10000 M3
G54
G0 X20 Y0
Z15
Z5
G41 D3.175
G1 Z0.635 F300
Z-6
Y20 F1000
X-20
Y-20
X20
Y0
G40
G0 Z15
M5
G28
M30
