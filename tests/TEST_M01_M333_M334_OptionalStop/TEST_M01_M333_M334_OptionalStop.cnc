G90 G94
G17
G21

(To run this test, run M334 from the console and play this file)
(The program should pause between each move)
(Then run M333 from the console and play the file again)
(It should run from start to end without any pauses)
(Optional stop mode will not persist on machine reset)

G01 X350
M01 (Optional Stop)
G01 X300
M01
G01 X350
M01
G01 X300
M01