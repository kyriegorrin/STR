#include <avr/io.h>
#include <util/delay.h>
#define BLINK_DELAY_MS 1000

/* Este codigo hace el blink de un LED 
   a base de hacer el toggle del pin
   asociado al LED (poniendo un uno). */

void setup() {
  // initialize digital pin 13 as an output// It is PB7 on Atmega2560 chip.
  DDRB |= _BV(DDB7);

}
void loop() {
  /* set pin 7 high to turn led on */
  PINB = _BV(PB7);
  _delay_ms(BLINK_DELAY_MS);
}

