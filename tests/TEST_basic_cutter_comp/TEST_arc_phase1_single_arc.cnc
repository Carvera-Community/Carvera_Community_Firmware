%
(Phase 1 Test: Single Isolated Arc with Compensation)
(Tests basic arc endpoint offset without corners)
(Expected: Smooth semicircle offset 3mm to the left)

G21 (metric)
G90 (absolute)
G17 (XY plane)
F1000 (feed rate 1000 mm/min)
(Move to arc start without compensation)
G40 (cancel any previous compensation)
G1 X0 Y0
(Enable left compensation with 6mm diameter tool = 3mm radius offset)
G41 D6
(Single semicircle arc - CW from X0 Y0 to X20 Y0)
(Center at X10 Y0, so I=10 J=0)
(With G41, arc should offset 3mm to the left/outward)
G2 X20 Y0 I10 J0
(Continue with linear move to see connection)
G1 X30 Y0
(Return without compensation)
G40
M2 (end program)
%
