G90 G94
G17
G21

M118 testing 1+1. Should return 2
M118.1 P[1+1]
M118 testing 2-1. Should return 1
M118.1 P[2-1]
M118 testing 2*3 Should return 6
M118.1 P[2*3]
M118 testing 5/4 Should return 1.25
M118.1 P[5/4]
M118 testing 2^3 Should return 8
M118.1 P[2^3]
M118 testing Pemdas [1+2*[5-2*2]+2^2+sin[0.5]]
M118 Should return 7.009
M118.1 P[1+2*[5-2*2]+2^2+sin[0.5]]
M118 testing sin[45] should return 0.707
M118.1 P [sin[45]] 
M118 testing sin[0] should return 0
M118.1 P [sin[0]]
M118 testing sin[90] should return 1
M118.1 P [sin[90]]
M118 testing cos[0] should return 1
M118.1 P [cos[0]]
M118 testing cos[45] should return 0.707
M118.1 P [cos[45]]
M118 testing cos[90] should return 0
M118.1 P [cos[90]]
M118 testing tan[0] should return 0
M118.1 P [tan[0]]
M118 testing tan[90] should return nan, which is undefined
M118.1 P [tan[90]]
M118 testing asin[0] should return 0 degrees
M118.1 P [asin[0]]
M118 testing asin[0.5] should return 30 degrees
M118.1 P [asin[0.5]]
M118 testing acos[0] should return 90
M118.1 P [acos[0]]
M118 testing acos[0.5] should return 60 degrees
M118.1 P [acos[0.5]]
M118 testing atan[0] should return 0 degrees
M118.1 P [atan[0]]
M118 testing atan[0.5] should return 26.565
M118.1 P [atan[0.5]]

M118 testing round[3.2] should return 3
M118.1 P [round[3.2]]
M118 testing round[3.5] should return 4
M118.1 P [round[3.5]]

M118 testing fix[3.2] should return 3
M118.1 P [fix[3.2]]
M118 testing fix[3.5] should return 3
M118.1 P [fix[3.5]]

M118 testing fup[3.2] should return 4
M118.1 P [fup[3.2]]
M118 testing fup[3.5] should return 4
M118.1 P [fup[3.5]]

M118 testing sqrt[4] should return 2
M118.1 P [sqrt[4]]
M118 testing sqrt[15] should return 3.873
M118.1 P [sqrt[15]]

M118 testing ln[1.2] should return 0.182
M118.1 P [ln[1.2]]

M118 testing abs[-2] should return 2
M118.1 P [abs[-2]]
M118 testing abs[2] should return 2
M118.1 P [abs[2]]

M118 testing exp[2] should return 7.389
M118.1 P [exp[2]]
M118 testing 3mod[2] should return 1
M118.1 P [3mod[2]] 

M118 testing 1and[2] should return 1
M118.1 P [1and[2]]
M118 testing 0or[0] should return 0
M118.1 P [0or[0]]
M118 testing 0xor[1] should return 1
M118.1 P [0xor[1]]
M118 testing 0nor[1] should return 0
M118.1 P [0nor[1]]

M118 testing 1eq[1] should return 1
M118.1 P [1eq[1]]
M118 testing 1ne[1] should return 0
M118.1 P [1ne[1]]
M118 testing 1lt[1] should return 0
M118.1 P [1lt[1]]
M118 testing 1le[1] should return 1
M118.1 P [1le[1]]
M118 testing 1gt[1] should return 0
M118.1 P [1gt[1]]
M118 testing 1ge[1] should return 1
M118.1 P [1ge[1]]

M118 testing 1eq[2] should return 0
M118.1 P [1eq[2]]
M118 testing 1ne[2] should return 1
M118.1 P [1ne[2]]
M118 testing 1lt[2] should return 1
M118.1 P [1lt[2]]
M118 testing 1le[2] should return 1
M118.1 P [1le[2]]
M118 testing 1gt[2] should return 0
M118.1 P [1gt[2]]
M118 testing 1ge[2] should return 0
M118.1 P [1ge[2]]

M118 testing 1eq[0] should return 0
M118.1 P [1eq[0]]
M118 testing 1ne[0] should return 1
M118.1 P [1ne[0]]
M118 testing 1lt[0] should return 0
M118.1 P [1lt[0]]
M118 testing 1le[0] should return 0
M118.1 P [1le[0]]
M118 testing 1gt[0] should return 1
M118.1 P [1gt[0]]
M118 testing 1ge[0] should return 1
M118.1 P [1ge[0]]

M118 testing set variable 101
#101 = 1.5
M118.1 P #101
M118 testing set variable 501
#501 = 1.5
M118.1 P#501
M118 testing get variable current tool number
M118.1 P#3026

M118 test pemdas with variable
M118 Should return 4.5
M118.1 P[1+2*[5-2*2]+#101]


M118 moving on to break tests, these will halt. Use goto line# to test each one. 
M600
M118 testing mismatched brackets v2
M118.1 P 1+2]+1
M600
M118 testing divide by zero error
M118.1 P [1/0]
M600
M118 testing mismatched brackets
M118.1 P [[1+2]
M600
M118 testing floating expression
M118.1 [P1+]
M600
M118 testing get variable that doesnt exist
M118.1 P#300
 