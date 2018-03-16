#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

/* Este programa hace servir la Input Capture Unit del Atmega2560
   para hacer saltar interrupciones en ambos flancos (ascendente y
   descendente) y gestionando via hardware el encendido del Timer1 
   para saber el tiempo que tardan las ondas generadas por nuestro
   ultrasonidos y poder calcular la distancia hasta el objeto.  */

double tiempo;
double distancia;

ISR (TIMER1_CAPT_vect) {
  if (PIND & 0x04) { //El evento fue un flanco ascendente
     TCCR1B |= (1 << CS11);
     Serial.print("Tiempo cuando flanco ascendente: ");
     Serial.println(ICR1);
  }
  else { //El evento fue un flanco descendente
    tiempo  = ICR1; //Cuando hay un evento el valor de TCNT1 se copia en ICR1!
    TCCR1B ^= (1 << CS11);
    TCNT1   = 0; //Reset del timer
    Serial.print("Tiempo cuando flanco descendente: ");
    Serial.println(ICR1);
  }
  
  TCCR1B ^= (1 << ICES1); //Toggle del flanco que se miraba

}

void setup() {
  Serial.begin(9600);

  //General I/O config
  //Pin para el trigger
  DDRE  |=  (1 << PE4); //Setting as output
  PORTE &= ~(1 << PE4); //Initial output = 0

  //Interrupt config (echo pin)
  DDRD  &=   ~(1 << PD4);  //PD4 es el pin ICP1 (input capture del timer 1), lo configuramos de input
  PORTD |= (1 << PORTD4);  //Pull-up resistor

  //Timer1 Input Capture U. configuracion
  TCCR1A  = 0; //Los bits COMx no tienen efecto sobre la unidad IC. y queremos que opere en modo Normal
  TCCR1B &= (1 << ICES1); //Flanco ascendente y prescaler de 8

  TCNT1 = 0;    

  //inicializacion variables
  tiempo = 0.0;
  distancia = 0.0; //Aunque ambas variables son doubles, no deben tomar valores negativos...

  sei();

}

void loop() {
  PORTE |= (1 << PE4); //Trigger HIGH
  _delay_us(15); //Wait minimum time
  PORTE &= ~(1 << PE4); //Trigger LOW
  
  //El timer está configurado para que cada clock sea de 0.5us, así
  //que tenemos que adaptar los calculos a ello.

  Serial.print("Valor ICP1: ");
  Serial.println(PIND & 0x04); 
  distancia = (tiempo/116.0);
  //Serial.println(distancia);
  _delay_ms(100);
}
