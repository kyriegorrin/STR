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

ISR (TIM4_CAPT_vect) {
	//Copia en el ICR el contenido del timer (diferencia entre inicio y fin), es decir, lo que ha durado el flanco
}

void setup() {
  Serial.begin(9600);

  //General I/O config
  //Pin para el trigger
  DDRE  |=  (1 << PE4); //Setting as output
  PORTE &= ~(1 << PE4); //Initial output = 0

  //Interrupt config (echo pin)
  DDRL &= ~(0b00000001);  //PL0 es el pin ICP4 (input capture del timer 4), lo configuramos de input
  //PORTL |=  (1 << PORTL0);  //Pull-up resistor
  
  DDRD &= ~(0b00000001); //INT0 pin set as input (TRIGGER_PIN); 
  PORTD |= (1 << PORTD0); //Turning on pull-up resistor


  //Timer1 Input Capture U. configuracion
  TCCR4A &= (1 << WGM41); //Los bits COMx no tienen efecto sobre la unidad IC. y queremos que opere en modo Normal
  TCCR4B &= (1 << ICNC4) | (1 << ICES4) | (1 << CS41) | (1 << WGM43); //Flanco ascendente
  TCNT4 = 0;    

  TIMSK4 &= (1 << ICIE4);

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

  distancia = (tiempo/116.0);
  //Serial.println(distancia);
  _delay_ms(100);
}
