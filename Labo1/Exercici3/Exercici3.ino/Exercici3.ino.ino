#include <avr/io.h>
#include <avr/interrupt.h>

/* El programa inicialitza un timer en mode CTC de tal manera que al arribar
   un cert valor el timer dispara un flag inclos en la nostra tabla d'interrupcions.
   Cada OCR1A cicles saltara la interrupcio que fa toggle del LED! */

ISR(TIMER1_COMPA_vect){
  
  //Toggle del led
  PINB = _BV(PB7);
  
}

void setup() {
  
  DDRB |= _BV(DDB7);  

  //Registres de configuracio a 0 inicialment
  TCCR1A = 0; 
  TCCR1B = 0;
  
  //Valor inicial timer
  TCNT1 = 0;
    
  //Prescaler a 256 (comenca timer) i en mode CTC
  TCCR1B |= _BV(CS12) | _BV(WGM12);

  //Valor que ha de fer match per saltar interrupcio
  OCR1A = 62500; 

  //Habilitem interrupts de timer en mode CTC
  TIMSK1 |= _BV(OCIE1A);

  //Inicialitzem interrupts globalment
  sei();  

}
void loop() {
  //WE ARE THE M A S T E R S    O F    A T M E G A

  //Manuel, no nos podemos ni fiar del datasheet ya que dice valores iniciales
  //de ciertos registros (como el TCCR1B) que son falsos.
}

