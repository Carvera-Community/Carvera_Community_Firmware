This test demonstrates the various 3 axis probing routines found in the firmware. 

To start the test, set up the machine bed as shown in the included images and put a 3 axis probe into the collet. 

Make sure your probe is communicating with the machine using the diagnostic menu

Be ready to press the Estop should anything go wrong. 

Run the probing routine without zprobe or auto leveling at 0,0 from anchor 1. It should start by probing the bed to set its Z height. 

The program will step through various probing cycles detailed below, printing to the MDI a report of the steps and pausing between each one. 


M460 H{known diameter} L{number of probe cycles to average over} I{is an angular offset to apply for each iteration.} Q{rotation to start probing cycle at}
probe a known bore multiple times and report back calculated probe tip diameter. 
The returned value will neeed to be saved to the config file with config-set sd zprobe.probe_tip_diameter #


Probe rectangular pocket/bore
M461 X{#} Y{#} H{#} Q{#} F{#} K{#} L{#} R{#} C{#} D{#} S{#}
You only need to provide X or Y as a parameter (you can provide both), all others are optional

X,Y : distance along the particular axis to probe as a radius. So to probe a roughly 100mm di bore, you would get the probe roughly centered and doM461 X60 Y60

H: Optional parameter, if set the probe will probe down by this value to find the pocket bottom and then retract slightly before probing the sides of the bore. Useful for shallow pockets

Q: Rotation of the pocket axes in relation to the X axis.
Ex. M461 X10 Q45 will probe along the positive 45 degree direction

F: optional fast feed rate override

L: setting L to 1 will repeat the entire probing operation from the newly found centerpoint

R: changes the retract distance from the edge of the pocket for the double tap probing

C: optional parameter, if H is enabled and the probe happens, this is how far to retract off the bottom surface of the part. Defaults to 2mm

D: Probe Tip Diameter, stored in config

S: save corner position as new WCS Zero in X and Y when this is set to 1

K: optional rapid feed rate override


Probe boss/web
M462 X{#} Y{#} H{#} Q{#} F{#} K{#} L{#} R{#} C{#} E{#} D{#} S{#}

You only need to provide X or Y as a parameter (you can provide both), all others are optional

X,Y : distance along the particular axis to probe as a radius. So to probe a roughly 100mm di boss, you would get the probe roughly centered and do M462 X60 Y60

H: Optional parameter, if set the probe will probe down by this value to find the pocket bottom and then retract slightly before probing the sides of the bore. Useful for shallow pockets

Q: Rotation of the pocket axes in relation to the X axis.
Ex. M462 X10 Q45 will probe along the positive 45 degree direction

F: optional fast feed rate override

L: setting L to 1 will repeat the entire probing operation from the newly found centerpoint

R: changes the retract distance from the edge of the pocket for the double tap probing

C: optional parameter, if H is enabled and the probe happens, this is how far to retract off the bottom surface of the part. Defaults to 2mm

E: how far below the top surface of the model to move down in order to probe on each side. D uses an optional probe routine, so if it hits a surface it will slightly retract before probing the x or y axis.

D: Probe Tip Diameter, stored in config

S: save corner position as new WCS Zero in X and Y when this is set to 1

K: optional rapid feed rate override



Probe internal corner
M463 X{} Y{} H{} F{} K{} L{} R{} C{} S{} D{}

You need to provide both X and Y as parameters, all others are optional. The values of X and Y determine probing direction, so to probe anchor 1 as an inside corner you need to input negative values for X and Y

X,Y : distance along the particular axis to probe

H: Optional parameter, if set the probe will probe down by this value to find the pocket bottom and then retract slightly before probing the sides of the bore. Useful for shallow pockets

F: optional fast feed rate override

K: optional rapid feed rate override

L: setting L to 1 will repeat the entire probing operation from the newly found centerpoint

R: changes the retract distance from the edge of the pocket for the double tap probing

C: optional parameter, if H is enabled and the probe happens, this is how far to retract off the bottom surface of the part. Defaults to 2mm

D: Probe Tip Diameter, stored in config

S: save corner position as new WCS Zero in X and Y



Probe external corner:
M464 X{} Y{} H{} F{} K{} L{} R{} C{} S{} H{} D{} E{}

You need to provide both X and Y as parameters, all others are optional. The values of X and Y determine probing direction, so to probe anchor 2 as an outside corner you need to input positive values for X and Y

-> |__
^

X,Y : distance along the particular axis to probe.

H: Optional parameter, if set the probe will probe down by this value to find the pocket bottom and then retract slightly before probing the sides of the bore. Useful for shallow pockets

F: optional fast feed rate override

K: optional rapid feed rate override

L: setting L to 1 will repeat the entire probing operation from the newly found centerpoint

R: changes the retract distance from the edge of the pocket for the double tap probing

C: optional parameter, if H is enabled and the probe happens, this is how far to retract off the bottom surface of the part. Defaults to 2mm

D: Probe Tip Diameter, stored in config

S: save corner position as new WCS Zero in X and Y

E: how far below the top surface of the model to move down in order to probe on each side. D uses an optional probe routine, so if it hits a surface it will slightly retract before probing the x or y axis.


Probe axis to find angle
M465 X{} Y{} H{} Q{} F{} K{} L{} R{} C{} S{} H{} D{} E{}

You must only provide one parameter from X or Y, not both.

^ ^
A C

X,Y : distance between the two points along the given axis to probe from. In the ascii it is the distance between A and C

H: Optional parameter, if set the probe will probe down by this value to find the pocket bottom and then retract slightly before probing the sides of the bore. Useful for shallow pockets

Q: Rotation of the pocket axes in relation to the X axis.
Ex. M465 X10 Q45 will rotate A-C 45 degrees and when probing it will probe perpendicular to this line.

F: optional fast feed rate override

K: optional rapid feed rate override

L: setting L to 1 will repeat the entire probing operation

R: changes the retract distance from the touched surface when double tapping the probe

C: optional parameter, if H is enabled and the Z probe happens, this is how far to retract off the bottom surface of the part. Defaults to 2mm

D: Probe Tip Diameter, stored in config

E: how far along the perpendicular direction to probe. In the ASCII, it is the distance between A or C and the angled surface it will touch.

