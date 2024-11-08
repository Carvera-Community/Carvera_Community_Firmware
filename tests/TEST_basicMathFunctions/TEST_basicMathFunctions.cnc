G90 G94
G17
G21

M118 testing 1+1. Should return 2
M118.1 P1+1
M118 testing 2-1. Should return 1
M118.1 P2-1
M118 testing 2*3 Should return 6
M118.1 P2*3
M118 testing 5/4 Should return 1.25
M118.1 P5/4
M118 testing 2^3 Should return 8
M118.1 P2^3
M118 testing Pemdas [1+2*[5-2*2]+2^2+SIN[0.5]]
M118 Should return 7.009
M118.1 P[1+2*[5-2*2]+2^2+SIN[0.5]]
M118 testing SIN[45] should return 0.707
M118.1 P SIN[45] 
M118 testing SIN[0] should return 0
M118.1 P SIN[0]
M118 testing SIN[90] should return 1
M118.1 P SIN[90]
M118 testing COS[0] should return 1
M118.1 P COS[0]
M118 testing COS[45] should return 0.707
M118.1 P COS[45]
M118 testing COS[90] should return 0
M118.1 P COS[90]
M118 testing TAN[0] should return 0
M118.1 P TAN[0]
M118 testing TAN[90] should return undefined
M118.1 P TAN[90]
M118 testing ASIN[0] should return 0 degrees
M118.1 P ASIN[0]
M118 testing ASIN[0.5] should return 30 degrees
M118.1 P ASIN[0.5]
M118 testing ACOS[0] should return 90
M118.1 P ACOS[0]
M118 testing ACOS[0.5] should return 60 degrees
M118.1 P ACOS[0.5]
M118 testing ATAN[0] should return 0 degrees
M118.1 P ATAN[0]
M118 testing ATAN[0.5] should return 26.565
M118.1 P ATAN[0.5]

M118 testing ROUND[3.2] should return 3
M118.1 P ROUND[3.2]
M118 testing ROUND[3.5] should return 4
M118.1 P ROUND[3.5]

M118 testing FIX[3.2] should return 3
M118.1 P FIX[3.2]
M118 testing FIX[3.5] should return 3
M118.1 P FIX[3.5]

M118 testing FUP[3.2] should return 4
M118.1 P FUP[3.2]
M118 testing FUP[3.5] should return 4
M118.1 P FUP[3.5]

M118 testing SQRT[4] should return 2
M118.1 P SQRT[4]
M118 testing SQRT[15] should return 3.873
M118.1 P SQRT[15]

M118 testing LN[1.2] should return 0.182
M118.1 P LN[1.2]

M118 testing ABS[-2] should return 2
M118.1 P ABS[-2]
M118 testing ABS[2] should return 2
M118.1 P ABS[2]

M118 testing EXP[2] should return 7.389
M118.1 P EXP[2]
M118 testing 3MOD[2] should return 1
M118.1 P 3MOD[2] 

M118 testing 1AND[2] should return 1
M118.1 P 1AND[2]
M118 testing 0OR[0] should return 0
M118.1 P 0OR[0]
M118 testing 0XOR[1] should return 1
M118.1 P 0XOR[1]
M118 testing 0NOR[1] should return 0
M118.1 P 0NOR[1]

M118 testing 1EQ[1] should return 1
M118.1 P 1EQ[1]
M118 testing 1NE[1] should return 0
M118.1 P 1NE[1]
M118 testing 1LT[1] should return 0
M118.1 P 1LT[1]
M118 testing 1LE[1] should return 1
M118.1 P 1LE[1]
M118 testing 1GT[1] should return 0
M118.1 P 1GT[1]
M118 testing 1GE[1] should return 1
M118.1 P 1GE[1]

M118 testing 1EQ[2] should return 0
M118.1 P 1EQ[2]
M118 testing 1NE[2] should return 1
M118.1 P 1NE[2]
M118 testing 1LT[2] should return 1
M118.1 P 1LT[2]
M118 testing 1LE[2] should return 1
M118.1 P 1LE[2]
M118 testing 1GT[2] should return 0
M118.1 P 1GT[2]
M118 testing 1GE[2] should return 0
M118.1 P 1GE[2]

M118 testing 1EQ[0] should return 0
M118.1 P 1EQ[0]
M118 testing 1NE[0] should return 1
M118.1 P 1NE[0]
M118 testing 1LT[0] should return 0
M118.1 P 1LT[0]
M118 testing 1LE[0] should return 0
M118.1 P 1LE[0]
M118 testing 1GT[0] should return 1
M118.1 P 1GT[0]
M118 testing 1GE[0] should return 1
M118.1 P 1GE[0]

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


M118 moving on to break tests. Use goto line# to test each one. 
M600
M118 testing mismatched brackets v2
M118.1 P 1+2]+1
M600
M118 testing divide by zero error
M118.1 P 1/0
M600
M118 testing mismatched brackets
M118.1 P [1+2
M600
M118 testing floating expression
M118.1 P1+
M600
M118 testing get variable that doesnt exist
M118.1 P#300
 