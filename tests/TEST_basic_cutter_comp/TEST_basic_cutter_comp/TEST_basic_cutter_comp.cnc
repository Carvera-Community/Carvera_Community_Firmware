G90 G54      ; Absolute coordinates
M491         ; Measure length and tool diameter
G0 X0 Y0
G41          ; Compensation left
G1 X50 F150 ; Should offset Y+diameter/2
G2 X100 Y50 I50 J0 ; Should increase arc radius
G1 Y100      ; Should offset X-diameter/2
G40          ; Cancel compensation
M2