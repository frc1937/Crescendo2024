/*
 *  And suddeenly,
 *  the penguin threw the apple out of the window.
 *  Breaking it in the proc.
 */

/*
 *        PORTS/PINS:
 *  REG | P |      IO
 *  PB0 | 5 | MISO/DATA_{OUT}
 *  PB1 | 6 |   MOSI/LED
 *  PB2 | 7 | SCKL/DATA_{IN}
 *  PB3 | 2 |     NC
 *  PB4 | 3 |     NC
 *  PB5 | 1 |   !RESET
 */

/*
 * TODO:
 * finish ASM code
 */

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

//register adr macros
#ifndef OFFSET
  #define OFFSET 0x20
#endif

#ifndef REG
  #define REG(addr)*((volatile unsigned char*)(addr+OFFSET))
#endif

#ifndef DDRB
  #define DDRB REG(0x17)
#endif

#ifndef PORTB
  #define PORTB REG(0x18)
#endif

#ifndef TCNT0
  #define TCNT0 0x32
#endif

#ifndef TCCR0B
  #define TCCR0B 0x33
#endif

#ifndef SREG
  #define SREG REG(0x3F)
#endif

#ifndef GIMSK
  #define GIMSK REG(0x3B)
#endif

#ifndef MCUCR
  #define MCUCR REG(0x35)
#endif

#define F_CPU 8000000 //  CPU clk
#define length 4  //  strip length

volatile uint8_t cmd;

int main(){
	// init
  cli();

  DDRB |= (_BV(DDB1) | _BV(DDB0));

  PORTB |= (_BV(PORTB1));

  TCCR0B &= ~((_BV(CS00)) | (_BV(CS01)) | (_BV(CS02)));  //  disables timer

  SREG |= 0b10000000;  //  SREG |= (_BV(I));

  GIMSK |= (_BV(INT0));

  MCUCR |= (_BV(ISC01) | _BV(ISC00)); //  sets INT0 trigger RISING

  sei();

  while(1){
    //  loop
    if(cmd > 3){
        PORTB |= (_BV(PORTB1));
      } else {
        PORTB &= ~(_BV(PORTB1));
      } 
  }
}

ISR(INT0_vect, ISR_NOBLOCK){
  if(MCUCR & ((1 << ISC00) == (1 << ISC00))){
    TCCR0B |= (_BV(CS01));  //  enable timer, prescaler=clk/8  --> 1MHz; so each cycle is one microsecond
    MCUCR &= ~(ISC00); //  sets INT0 trigger FALLING
  } else {
    cmd = TCNT0;
    TCCR0B &= ~(_BV(CS01));  //  disables timer
    TCNT0 = 0;  //  clear timer register
    MCUCR |= (ISC00);  // sets INT0 trigger RISING
  }
}

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

__asm__(
  ;r16 is for HIGH PB0
  ;r17 is for LOW PB0
  
  init:
    ldi r16, 0b00000001 ;consider using bit masks
    ldi r17, 0
    ret
  LOW:
    out PORTB, r16
    nop
    nop
    out PORTB, r17
    nop
    nop
    nop
    nop
    nop
    ret
  HIGH:
    out PORTB, r16
    nop
    nop
    nop
    nop
    nop
    out PROTB, r17
    nop
    nop
    ret
);
