; Test cutter compensation implementation
; Using explicit D word for tool diameter
; Tool diameter set to 6mm for testing

G90 ; Absolute positioning
G21 ; Millimeters

; Test 1: Simple line with left compensation
G0 X0 Y0 Z5    ; Move to start position
G41 D6         ; Enable left compensation with 6mm tool
G1 F150        ; Set feed rate
G1 X0 Y0       ; Move to start position of cut with offset
G1 X50 Y0      ; Should offset 3mm to left
G1 X50 Y50     ; Move to next position
G1 X0 Y50      ; Complete the rectangle
G1 X0 Y0       ; Return to start with offset
G40            ; Cancel compensation