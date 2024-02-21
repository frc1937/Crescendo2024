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
 *  OPTIMIZATION:
 *  ffs, make sure you optimize reg writes
 *  operations with bitwise operators.
 *  for setting bit LOW:  REG &= ~(_BW(bit));
 *  for setting bit HIGH: REG |= (_BW(bit));
 */

/*
 * TODO:
 *  -1   make a routine for counting microseconds from rise to fall of signal.
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

#define F_CPU 8000000 //  CPU clk
#define length 4  //  strip length
#define cdiv  //  the divisor for the count

volatile uint16_t count = 0;
	
int main(void){
	// init
  DDRB |= (_BW(DDB1) | _BW(DDB0));

  PORTB |= (_BW(PORTB1));
  PORTB &= ~(_BW(PORTB0));

  SREG |= (_BW(I));

  GIMSK |= (_BW(INT0));

  MCUCR |= (_BW(ISC01) | _BW(ISC00)); //  sets INT0 trigger RISING

	while(1){
		//  loop
    
		}
}

/*
*   changed boolean:
*   consider using a boolean to indicate to the main loop
*   wether the command has changed
*/
ISR(INT0_vect, ISR_NOBLOCK){
if((MCUCR & (1 << ISC00)) == (1 << ISC00)){
  rising();
  } else {
  falling();
  }
}

void rising(){
  MCUCR &= ~(_BW(ISC00)); //  sets INT0 trigger FALLING
  count++;
}

void falling() {
  MCUCR |= (_BW(ISC00));  // sets INT0 trigger RISING
}
