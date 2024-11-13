(Probing Playground)
G90 G94
G17
G21
G54
(run this file with the setup shown in the readme section)
(Use anchor 1, 0,0 as the starting position.)

M118 Start with a probe at the center position to set z height

G28
G0 X132 Y77
G38.2 Z-120 F500
G10 L20 Z0 P0
G0 Z20


(goto internal corner 1 position)
G0 Z100
G0 X100 Y50
G0 Z12.5

M118 probing internal corner with M463 X-10 Y-10
M118 values determine the direction the probe toward the surface
M463 X-10 Y-10
G4 S1 (pause for a second)

M118 for corners we have 2 new variables to play with. 
M118 lets go to the corner position at #153,#155
#154 (corner x pos, bore/boss center x)
#155 (corner y pos, bore/boss center y)
M600
G0 Z18
G53 G0 X#154 Y#155 (using absolute world coordinates as that is how the variables are stored right now)
G54
G4 S1 (pause for a second)

M600

(goto external corner 1 position)
G0 Z20
G0 X83 Y36
M118 probing external corner with M464 X10 Y10 H10
M118 values determine the direction the probe toward the surface
M464 X10 Y10 H10
G4 S1 (pause for a second)

G53 G0 X#154 Y#155 (using absolute world coordinates as that is how the variables are stored right now)
G54
G4 S1 (pause for a second)
M600


(goto external corner 2 position)
G0 Z20
G0 X171.5 Y35
M118 probing external corner with M464 X-10 Y10 C8
M118 values determine the direction the probe toward the surface
M464 X-10 Y10 C8
G4 S1 (pause for a second)

G53 G0 X#154 Y#155 (using absolute world coordinates as that is how the variables are stored right now)
G54
G4 S1 (pause for a second)
M600

(goto external corner 3 position)
G0 Z20
G0 X184.5 Y120
G0 Z13
M118 probing external corner with M464 X-10 Y-10 H12 C2
M118 values determine the direction the probe toward the surface
M464 X-10 Y-15 H12 C2
G4 S1 (pause for a second)

G53 G0 X#154 Y#155 (using absolute world coordinates as that is how the variables are stored right now)
G54
G4 S1 (pause for a second)
M600

(goto internal corner 3 position)
G0 Z20
G0 X170 Y105
G0 Z8

M118 probing internal corner with M463 X10 Y10 H10 R3
M118 values determine the direction the probe toward the surface
M463 X10 Y10 H10 R3
G4 S1 (pause for a second)

G0 Z18
G53 G0 X#154 Y#155 (using absolute world coordinates as that is how the variables are stored right now)
G54
G4 S1 (pause for a second)

M600

(goto external corner 4 position)
G0 Z20
G0 X82 Y122

M118 probing external corner with M464 X10 Y-10 H8 C2
M118 values determine the direction the probe toward the surface
M464 X10 Y-10 H8 C2
G4 S1 (pause for a second)

G53 G0 X#154 Y#155 (using absolute world coordinates as that is how the variables are stored right now)
G54
G4 S1 (pause for a second)
M600

(goto center rectangle probe position)
G0 Z20
G0 X132 Y77
G0 Z7

M118 Probing center position of internal rectangle with M461 X15 Y15 H12 C4
M118 the H command starts by probing down and retracting slightly if a surface is found. Otherwise it continues as normal
M461 X50 Y50 H10 C3.5
G4 S1 (pause for a second)
M600


(goto center rectangle probe position)
G0 Z20
G0 X132 Y77

M118 measuring a boss with M462 X125/2 Y110/2 E16 F400
(we are doing some basic math in this one)
(E is the distance below the top surface to probe, it combines with C)
M462 X125/2 Y110/2 E16 F400
G4 S1 (pause for a second)

M600


(goto center web position)
G0 Z20
G0 X84 Y79

M118 Measuring web with M462 X10 E6 F400
M118 M461 nd M462 can both operate on X, Y or both
M462 X10 E6 F400
G4 S1 (pause for a second)

M600


(goto angle start position)
G0 Z20
G0 X4 Y15
G0 Z3

M118 Measuring an angle with M465 X30 E30 R1
M465 X15 E30 R1
G4 S1 (pause for a second)
(this uses a new variable)
#153 (angle of the surface in degrees)
M600
G0 X20 Y20
M465 Y15 E-30 R1
M465 Y15 E-30 R1 Q-45

G0 X4 Y15
G0 Z3

M118 Measuring an angle with M465 X15 E30 R1 Q45 V0
M465 X15 E30 R1 Q45 V0
G4 S1 (pause for a second)
(this uses a new variable)
#153 (angle of the surface in degrees)
M600

(goto bore start position)
G0 Z20
G0 X247 Y151
G0 Z4

M118 probe bore with angle using M461 X15 Y15 Q45 K600 S1
M118 K value is rapid feedrate
M461 X15 Y15 K600 L2
G4 S1 (pause for a second)
M600

M118 Calibrating Probe Tip With M460 X25 L3 I20
M118 where the I parameter offsets the probing on each repeat
M460 X25 L3 I20
G4 S1 (pause for a second)
M600

M118 probe bore with angle using M461 X15 Y15 Q45 K600 S1
M118 K value is rapid feedrate
M118 S parameter sets the WCS zero to the center of the bore.
M461 X15 Y15 Q45 K600 S1
G4 S1 (pause for a second)
M600


M5
M30
