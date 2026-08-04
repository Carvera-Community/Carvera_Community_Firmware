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

M118 while: condition false at start - should print nothing inside, 0 after
#101 = 10
O201 while [#101 lt 5]
  M118.1 P999
O201 endwhile
M118.1 P0

; --- do / while ---

M118 do-while: always executes at least once - should print 10 once then stop
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

M118 repeat: count is 0 - should print nothing inside, 0 after
#101 = 1
O401 repeat [0]
  M118.1 P999
O401 endrepeat
M118.1 P0

; --- break ---

M118 break: exit before 4th iteration - should print 1, 2, 3
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

; --- complex expressions in conditions ---

M118 if: arithmetic + comparison - should print 1
#101 = 10
#102 = 6
O900 if [[#101 + #102] gt sqrt[144]]
  M118.1 P1
O900 endif

M118 if: and of two comparisons - should print 1
#101 = 5
#102 = 8
O901 if [[#101 gt 0] and [#102 lt 10]]
  M118.1 P1
O901 endif

M118 if: and false - should print 0 after
#101 = 0
#102 = 99
O902 if [[#101 gt 0] and [#102 lt 10]]
  M118.1 P999
O902 endif
M118.1 P0

M118 if: or saves false-and - should print 1
#101 = 0
#102 = 3
O903 if [[#101 gt 0] or [#102 lt 10]]
  M118.1 P1
O903 endif

M118 elseif: product comparison - should print 2
#101 = 3
#102 = 15
O904 if [#101 lt 0]
  M118.1 P1
O904 elseif [[#101 * #102] le 50]
  M118.1 P2
O904 else
  M118.1 P3
O904 endif

M118 elseif: nested pemdas - should print 2
#101 = 1
O905 if [[1 + 2 * [#101 + 1] + 2^2] eq 11]
  M118.1 P1
O905 elseif [[1 + 2 * [#101 + 1] + 2^2] eq 9]
  M118.1 P2
O905 else
  M118.1 P3
O905 endif

M118 if: variable subtraction vs power - should print 1
#101 = 10
#102 = 4
O906 if [[#101 - #102] ge sqrt[#102 * #102]]
  M118.1 P1
O906 endif

M118 while: and with trig - should print 1, 2, 3, 4, 5, 6
#101 = 1
O910 while [[#101 le 6] and [sin[30] gt 0.4]]
  M118.1 P#101
  #101 = [#101 + 1]
O910 endwhile

M118 while: mod and upper bound - should print 3, 6, 9
#101 = 1
O911 while [#101 le 10]
  O911 if [[#101 mod 3] eq 0]
    M118.1 P#101
  O911 endif
  #101 = [#101 + 1]
O911 endwhile

M118 repeat: expression count - should print 1, 2, 3, 4
#101 = 1
#102 = 2
#103 = 5
O912 repeat [[#103 - #102 + 1]]
  M118.1 P#101
  #101 = [#101 + 1]
O912 endrepeat

M118 do-while: power limit - should print 1, 2, 3, 4
#101 = 1
O913 do
  M118.1 P#101
  #101 = [#101 + 1]
O913 while [#101 le 2^2]

M118 while: abs/mod compound - should print 3, 4, 5, 10
#101 = 1
O914 while [#101 le 10]
  O915 if [[abs[#101 - 4] lt 2] or [[#101 mod 5] eq 0]]
    M118.1 P#101
  O915 endif
  #101 = [#101 + 1]
O914 endwhile

M118 continue: xor filter - should print 2, 3, 5, 7
#101 = 1
O920 while [#101 le 8]
  O921 if [[[#101 mod 2] eq 0] xor [fix[#101 / 3] gt 0]]
    M118.1 P#101
  O921 else
    #101 = [#101 + 1]
    O920 continue
  O921 endif
  #101 = [#101 + 1]
O920 endwhile

M118 nested if: perfect squares - should print 4, 9
#101 = 1
O930 while [#101 le 10]
  O931 if [[[round[sqrt[#101]]^2] eq #101] and [#101 gt 3]]
    M118.1 P#101
  O931 endif
  #101 = [#101 + 1]
O930 endwhile

O940 sub
  M118 sub expr args:
  M118.1 P#1
  M118.1 P#2
O940 endsub

M118 call sub with expressions - should print 5, 6
#101 = 7
#102 = 3
O940 call [#101 - 2] [#102 * 2]

M118 call sub with nested expr - should print 8, 2
#101 = 4
O940 call [#101 * 2] [sqrt[#101]]

; --- break / continue: O-number targeting ---

M118 break outer loop: should print 1, 2
#101 = 1
O950 while [#101 le 5]
  #102 = 1
  O951 while [#102 le 5]
    O950 if [#101 gt 2]
      O950 break
    O950 endif
    O951 if [#102 eq 1]
      M118.1 P#101
    O951 endif
    #102 = [#102 + 1]
  O951 endwhile
  #101 = [#101 + 1]
O950 endwhile

M118 continue outer loop: should print 1, 3, 5
#101 = 1
O960 while [#101 le 5]
  #102 = 1
  O961 while [#102 le 5]
    O960 if [[#101 mod 2] eq 0]
      #101 = [#101 + 1]
      O960 continue
    O960 endif
    O961 if [#102 eq 1]
      M118.1 P#101
    O961 endif
    #102 = [#102 + 1]
  O961 endwhile
  #101 = [#101 + 1]
O960 endwhile

M118 break inner loop: should print 1, 1, 1, 2, 2, 2, 3, 3, 3
#101 = 1
O970 while [#101 le 3]
  #102 = 1
  O971 while [#102 le 5]
    O971 if [#102 gt 3]
      O971 break
    O971 endif
    M118.1 P#101
    #102 = [#102 + 1]
  O971 endwhile
  #101 = [#101 + 1]
O970 endwhile

; --- false while with inner do-while (skip_to depth) ---

M118 while false with inner do-while: should print 0 after
#101 = 10
O1000 while [#101 lt 5]
  O1001 do
    M118.1 P999
    #101 = [#101 + 1]
  O1001 while [#101 lt 3]
O1000 endwhile
M118.1 P0

M118 while false with do-while and nested while: should print 0 after
#101 = 10
O1010 while [#101 lt 5]
  O1011 do
    O1012 while [1]
      M118.1 P999
    O1012 endwhile
  O1011 while [1]
O1010 endwhile
M118.1 P0

; --- continue in do-while re-evaluates condition ---

M118 continue in do-while checks condition: should print 2, 4
#101 = 0
O1020 do
  #101 = [#101 + 1]
  O1021 if [[#101 mod 2] eq 1]
    O1020 continue
  O1021 endif
  M118.1 P#101
O1020 while [#101 lt 5]

; --- break from while containing do-while ---

M118 break from while with inner do-while: should print 1, 2
#101 = 1
O1030 while [#101 le 5]
  O1031 do
    #102 = 1
  O1031 while [0]
  O1032 if [#101 ge 3]
    O1030 break
  O1032 endif
  M118.1 P#101
  #101 = [#101 + 1]
O1030 endwhile

M2
