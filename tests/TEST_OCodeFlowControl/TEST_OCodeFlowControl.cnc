G90 G94
G17
G21

; -----------------------------------------------------------------------
; O-code flow control tests
; Each section prints expected output via M118 then the actual result.
; -----------------------------------------------------------------------

; --- if / elseif / else / endif ---

M118 if: condition true - should print 1
#101 = 5
O100 if [#101 gt 3]
  M118.1 P1
O100 endif

M118 if: condition false - should print nothing inside, 0 after
#101 = 1
O101 if [#101 gt 3]
  M118.1 P999
O101 endif
M118.1 P0

M118 elseif: second branch - should print 2
#101 = 5
O102 if [#101 lt 3]
  M118.1 P1
O102 elseif [#101 lt 10]
  M118.1 P2
O102 else
  M118.1 P3
O102 endif

M118 else: last branch - should print 3
#101 = 20
O103 if [#101 lt 3]
  M118.1 P1
O103 elseif [#101 lt 10]
  M118.1 P2
O103 else
  M118.1 P3
O103 endif

; --- while / endwhile ---

M118 while: count to 3 - should print 1, 2, 3
#101 = 1
O200 while [#101 le 3]
  M118.1 P#101
  #101 = [#101 + 1]
O200 endwhile

M118 while: condition false at start - should print nothing
#101 = 10
O201 while [#101 lt 5]
  M118.1 P999
O201 endwhile
M118.1 P0

; --- do / while ---

M118 do-while: always executes at least once - should print 1 then stop
#101 = 10
O300 do
  M118.1 P#101
  #101 = [#101 + 1]
O300 while [#101 lt 5]

; --- repeat / endrepeat ---

M118 repeat: execute 3 times - should print 1, 2, 3
#101 = 1
O400 repeat [3]
  M118.1 P#101
  #101 = [#101 + 1]
O400 endrepeat

M118 repeat: count is 0 - should print nothing
#101 = 1
O401 repeat [0]
  M118.1 P999
O401 endrepeat
M118.1 P0

; --- break ---

M118 break: exit loop early at 3 - should print 1, 2, 3
#101 = 1
O500 while [#101 le 10]
  O501 if [#101 gt 3]
    O500 break
  O501 endif
  M118.1 P#101
  #101 = [#101 + 1]
O500 endwhile

; --- continue ---

M118 continue: skip even numbers - should print 1, 3, 5
#101 = 1
O600 while [#101 le 5]
  O601 if [[#101 mod 2] eq 0]
    #101 = [#101 + 1]
    O600 continue
  O601 endif
  M118.1 P#101
  #101 = [#101 + 1]
O600 endwhile

; --- sub / endsub / call ---

O700 sub
  M118 in subroutine - arg 1 and 2:
  M118.1 P#1
  M118.1 P#2
O700 endsub

M118 call sub with [10] [20] - should print 10, 20
O700 call [10] [20]

M118 call sub again with [42] [7] - should print 42, 7
O700 call [42] [7]

; --- sub: verify #1-#30 are restored after call ---

#1 = 99
O710 sub
  #1 = 0
O710 endsub
O710 call
M118 after call, #1 should be restored to 99
M118.1 P#1

; --- nested if inside while ---

M118 nested if in while: should print 2, 4
#101 = 1
O800 while [#101 le 5]
  O801 if [[#101 mod 2] eq 0]
    M118.1 P#101
  O801 endif
  #101 = [#101 + 1]
O800 endwhile

M2
