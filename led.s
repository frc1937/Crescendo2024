/*
* WS2812B PROTOCOL:
* code 0: 0.4us HIGH, 0.85us LOW
* code 1: 0.8us HIGH, 0.45us LOW
*
* time to clock cycles:
* 0.4us = 3.2   --> 3 = 0.375us (err:+25ns)
* 0.85us = 6.8  --> 7 = 0.875us (err:-25ns)
* 0.8us = 6.4   --> 6 = 0.75us  (err:-50ns)
* 0.45us = 3.6  --> 4 = 0.5us   (err:+50ns)
*/

_asm__(
  sendByte:
    ldi r31, 8  ; set counter to 8
re: rol r30  ; set carry flag to MSB
    brcs HIGH   ; if carry is 1 goto subruotine HIGH
    rcall LOW   ; else goto subruotine LOW
    dec r31     ; r31--
    brne re     ; if r31 != 0 repeat 
    rcall RES   ; reste leds
    ret
  LOW:
    sbi PORTB, 0  ; 2clks
    ; 375ns HIGH: 3clks
    nop           ; 1clks
    cbi PORTB, 0  ; 2clks
    ; 875ns LOW: 7clks
    nop           ; 1clks
    nop           ; 1clks
    nop           ; 1clks
    ret           ; 4clks
  HIGH:
    sbi PORTB, 0  ; 2clks
    ;750ns: 6clks
    nop           ; 1clks
    nop           ; 1clks
    nop           ; 1clks
    nop           ; 1clks
    cbi PORTB, 0  ; 2clks
    ret           ; 4clks
  RES:
    cbi PORTB, 0
    ret
);
//please use timer for RES ffs
