G90 G94
G17
G21

T1 M6
S1000 M3
G54

G0 X100

(testing if the tool break test handles a situation that should never happen.)
(The post processor should have added a M5 before this point)
(the machine should not throw an error on this command unless the tool broke)
M491.1 H0.1


(paused to allow the user to remove replace the tool in the spindle with a broken one)
(using M490.2, M490.1. without changing tool number or recalibtating)
(to simulate a broken tool)
(after this point the machine should throw an error)
M600
M491.1