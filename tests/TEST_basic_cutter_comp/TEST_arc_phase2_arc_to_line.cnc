%
(Phase 2 Test: Arc to Line and Line to Arc Transitions)
(Tests corner intersection calculation between arcs and lines)
(Expected: Smooth transitions with correct corner handling)

G21 (metric)
G90 (absolute)
G17 (XY plane)
F1000

G0 X-10 Y-10 Z5
G1 Z0

(Test 1: Arc followed by Line)
G40
G1 X0 Y0
G41 D6

(Quarter circle CW from X0,Y0 to X10,Y10)
(Center at X10,Y0, so I=10 J=0)
G2 X10 Y10 I10 J0

(Straight line - should calculate intersection with arc)
G1 X30 Y10

(Return)
G40
G1 Y0
G1 X0

M0 (pause for inspection)

(Test 2: Line followed by Arc)
G41 D6

(Straight line)
G1 X0 Y20
G1 X10 Y20

(Quarter circle CCW from X10,Y20 to X20,Y30)
(Center at X20,Y20, so I=10 J=0)
G3 X20 Y30 I10 J0

(Continue with line)
G1 Y40

(Return)
G40
G1 X0 Y40
G1 Y20
G1 X0 Y0

G0 Z5
M2
%
