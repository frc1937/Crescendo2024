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
