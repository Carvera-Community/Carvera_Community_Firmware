; Test cutter compensation implementation
; Using explicit D word for tool diameter
; Tool diameter set to 6mm for testing

G90 ; Absolute positioning
G21 ; Millimeters

; Test 1: Simple line with left compensation
G0 X0 Y0 Z5    ; Move to start position
G41 D6         ; Enable left compensation with 6mm tool
G1 F1000       ; Set feed rate
G1 X50 Y0      ; Should offset 3mm to left
G40            ; Cancel compensation

; Test 2: Simple line with right compensation  
G0 X0 Y0       ; Return to start
G42 D6         ; Enable right compensation
G1 X50 Y0      ; Should offset 3mm to right
G40            ; Cancel compensation

; Test 3: Outside corner with left compensation
G0 X0 Y0       ; Return to start
G41 D6         ; Enable left compensation
G1 X50 Y0      ; First line
G1 X50 Y50     ; 90 degree corner, should maintain offset
G40

; Test 4: Inside corner with right compensation
G0 X0 Y0       ; Return to start
G42 D6         ; Enable right compensation
G1 X50 Y0
G1 X50 Y-50    ; Inside corner, should maintain offset
G40

; Test 5: CW arc with left compensation
G0 X0 Y0
G41 D6
G2 X50 Y0 I25 J0  ; Should increase arc radius by 3mm
G40

; Test 6: CCW arc with right compensation
G0 X0 Y0
G42 D6
G3 X50 Y0 I25 J0  ; Should decrease arc radius by 3mm
G40


; Simple Lines (Tests 1-2)
; 
; When cutting a straight line with G41, the path should be offset 3mm (half tool diameter) to the left
; With G42, it should be offset 3mm to the right
; Use calipers or measure on the machine's DRO to verify the offset
; Corner Compensation (Tests 3-4)
; 
; Outside corners (Test 3) ;should maintain constant offset around the corner
; Inside corners (Test 4) ;should also maintain proper offset
; Watch the machine movement - it should smoothly transition around corners
; Arc Compensation (Tests 5-6)
; 
; For G2 (CW) ;with G41, the arc radius should increase by 3mm
; For G3 (CCW) ;with G42, the arc radius should decrease by 3mm
; You can measure from arc center to verify radius change