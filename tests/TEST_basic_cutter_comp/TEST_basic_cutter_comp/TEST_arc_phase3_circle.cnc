%
(Phase 3 Test: Arc to Arc - Complete Circle)
(Tests corner intersection between two arcs)
(This is the original failing test case)
(Expected: Two semicircles form perfect compensated circle)

G21 (metric)
G90 (absolute)
G17 (XY plane)
F1000

G0 X-15 Y0 Z5
G1 Z0

(Move to start without compensation)
G40
G1 X0 Y0

(Enable left compensation - 6mm diameter = 3mm radius)
G41 D6

(First semicircle: CW from origin to X14.142,Y-14.142)
(This is northeast to southeast quadrant)
(Center at X7.071,Y-7.071 from start, so I=7.071 J=-7.071)
(Original radius ~10mm, compensated should be ~13mm)
G2 X7.071 Y-7.071 I7.071 J-7.071

(Second semicircle: continues CW back to origin)
(Center at X-7.071,Y7.071 from current position)
(These two arcs should form a complete offset circle)
G2 X-7.071 Y7.071 I-7.071 J7.071

(Exit with line to verify connection)
G1 X-10 Y0

(Cancel compensation and return)
G40
G1 X-15 Y0
G0 Z5

M2
%
