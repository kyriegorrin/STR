#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

//Este programa calcula i escribe por serial la distancia entre el
//sensor HRC04 y la superficie que tenga delante. Para eso, necesitamos
//configurar y usar interrupciones externas correctamente. Cuando mandamos un pulso
//al sensor, debemos controlar los flancos que nos llegan por el pin asignado a INT0.
//En caso de flanco ascendednte, iniciamos timer. En caso de ser descendente, paramos el
//timer, cogemos el valor y lo usamos para hacer nuestros cálculos de distancia.
//Finalmente, reseteamos el contenido del timer. Nuestro loop simplemente muestra el
//valor resultante de los cálculos por serial.

//Variables auxiliars
unsigned int timer1Val;
double distancia;

/* DESCRIPCIO DE PINS */
//TRIGGER: 1(TX0 arduino)
//ECHO: 21 (SCL arduino)

ISR(INT0_vect){
  //Testing
  if(PIND & 0x01){//Enxufa timer
    //Prescaler a 8, posem en marxa timer
    TCCR1B |= (1 << CS11);
  }else{//Desconecta a l'avia
    //Parem el timer (canviar prescaler a 0 para el timer)
    TCCR1B = 0;    
    timer1Val = TCNT1;
    TCNT1 = 0; //Reset de contenido del timer
  }
}

void setup() {
  //Enabling serial communication
  Serial.begin(9600);

  //General I/O config
  //Trigger pin
  DDRE |= (1 << PE4); //Setting as output
  PORTE &= ~(1 << PE4); //Initial output = 0

  //Interrupt config (echo pin)
  DDRD &= ~(0b00000001); //INT0 pin set as input (TRIGGER_PIN);
  PORTD |= (1 << PORTD0); //Turning on pull-up resistor

  EICRA |= (1 << ISC00); //Setting interrupt trigger on ANY logic change
  EIMSK |= (1 << INT0); //Turning on INT0

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

void loop() {
  //PORTE &= ~(1 << PE0);
  PORTE |= (1 << PE4); //Trigger HIGH
  _delay_us(15); //Wait minimum time
  PORTE &= ~(1 << PE4); //Trigger LOW
  //El timer está configurado para que cada clock sea de 0.5us, así
  //que tenemos que adaptar los calculos a ello.
  distancia = (timer1Val/116.0);
  Serial.println(distancia);
  _delay_ms(100);  
}

