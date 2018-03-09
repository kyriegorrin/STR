#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#define BLINK_DELAY_MS 1000


ISR(TIMER1_COMPA_vect){
  //Toggle del led
  PORTB ^=_BV(PB7);
  Serial.println("MEME");
  
}

void setup() {
  // initialize digital pin 13 as an output// It is PB7 on Atmega2560 chip.
  DDRB |= _BV(DDB7);

  // set up timer with prescaler = 64 and CTC mode
    TCCR1B |= (1 << WGM12)|(1 << CS11)|(1 << CS10);

    //TCCR1A |= (1 << COM1A0);
  
    // initialize counter
    TCNT1 = 0;
  
    // initialize compare value
    OCR1A = 24999;
  
    // enable compare interrupt
    TIMSK1 |= (1 << OCIE1A);
  
    // enable global interrupts
    sei();

  

  Serial.begin(9600);
}
void loop() {
  //WE ARE THE M A S T E R S    O F    A T M E G A
  Serial.println(TCNT1);
}

