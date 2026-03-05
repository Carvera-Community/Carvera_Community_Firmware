; Test cutter compensation implementation
; Using explicit D word for tool diameter
; Tool diameter set to 6mm for testing

G90 ; Absolute positioning
G21 ; Millimeters

; Test 1: Simple line with left compensation
G0 X0 Y0 Z5    ; Move to start position
G41 D3.175         ; Enable left compensation with 6mm tool diameter
G1 X0 Y0 F500       ; Move to start position of cut with offset  Set feed rate
X50       ; Should offset 3mm to left
Y50     ; Move to next position
X0       ; Complete the rectangle
Y0       ; Return to start with offset
G40            ; Cancel compensation