#include "HRC04.h"

//TRIGGER_PIN 1
//ECHO_PIN 19 -> TO FIX INTO THE CODE

//Variables auxiliars
unsigned int timer1Val;
double distancia;

//Rutina de servei d'interrupció PROBABLEMENT S'HA DE TUNEJAR
ISR(INT2_vect){
  //Testing
  if(PIND & 0x04){//Enxufa timer
    //Prescaler a 8, posem en marxa timer
    TCCR1B |= (1 << CS11);
  }else{//Desconecta a l'avia
    //Parem el timer (canviar prescaler a 0 para el timer)
    TCCR1B = 0;    
    timer1Val = TCNT1;
    TCNT1 = 0; //Reset de contenido del timer
  }
}

HRC04::HRC04() {
  //General I/O config
  //Trigger pin
  DDRE |= (1 << PE4); //Setting as output
  PORTE &= ~(1 << PE4); //Initial output = 0

  //Interrupt config (echo pin)
  DDRD &= ~(0b00000100); //INT0 pin set as input (TRIGGER_PIN);
  //PORTD |= (1 << PORTD0); //Turning on pull-up resistor

  EICRA |= (1 << ISC20); //Setting interrupt trigger on ANY logic change
  EIMSK |= (1 << INT2); //Turning on INT0

  //TIMERS CONFIG
  //Registres de configuracio a 0 inicialment
  TCCR1A = 0; 
  TCCR1B = 0;
  
  //Valor inicial timer
  TCNT1 = 0;    

  //Valor auxiliar per a calculs inicialitzat a 0
  timer1Val = 0;
    
  sei();  //Enabling global interrupts
}

HRC04::~HRC04(){/*L'avia morta que la tracti algu altre*/}

double HRC04::getDistancia() {
  //PORTE &= ~(1 << PE0);
  PORTE |= (1 << PE4); //Trigger HIGH
  _delay_us(15); //Wait minimum time
  PORTE &= ~(1 << PE4); //Trigger LOW
  //El timer está configurado para que cada clock sea de 0.5us, así
  //que tenemos que adaptar los calculos a ello.
  distancia = 10*timer1Val/116;
  _delay_ms(10); 
  return distancia; 
}

