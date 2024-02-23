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
 *  ~0   check if all register macros work
 *  ~1   make a routine for counting microseconds from rise to fall of signal using timer.
 *  -2   check for race condition
 *  -3a  try fastLED or adafruit lib
 *  -3b  if that doesn't work make ur own(git gud son)
 *  -4   make code for leds using lib
 *  *    OPTIMIZE!!!
 *  **   due 28/2/2024
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
  DDRB |= (_BV(DDB1) | _BV(DDB0));

  PORTB |= (_BV(PORTB1));
  PORTB &= ~(_BV(PORTB0));

  TCCR0B &= ~((_BV(CS00)) | (_BV(CS01)) | (_BV(CS02)));  //  disables timer

  SREG = 128;  //  SREG |= (_BV(I));

  GIMSK |= (_BV(INT0));

  MCUCR |= (_BV(ISC01) | _BV(ISC00)); //  sets INT0 trigger RISING

  while(1){
		//  loop
      if(cmd > 3){
          PORTB |= (_BV(PORTB1));
       } else {
          PORTB &= ~(_BV(PORTB1));
       } 
  }
}

/*
*   changed boolean:
*   consider using a boolean to indicate to the main loop
*   wether the command has changed
*/
ISR(INT0_vect, ISR_NOBLOCK){
if(MCUCR & ((1 << ISC00) == (1 << ISC00))){
  rising();
  } else {
  falling();
  }
}

void rising(){
  TCCR0B |= (_BV(CS00) | (_BV(CS01)));  //  enable timer, prescaler=clk/8  --> 1MHz; so each cycle is one microsecond
  MCUCR &= ~(ISC00); //  sets INT0 trigger FALLING
}

void falling() {
  cmd = TCNT0;
  TCCR0B &= ~(_BV(CS00) | (_BV(CS01)));  //  disables timer
  TCNT0 = 0;  //  clear timer register
  MCUCR |= (ISC00);  // sets INT0 trigger RISING
}
