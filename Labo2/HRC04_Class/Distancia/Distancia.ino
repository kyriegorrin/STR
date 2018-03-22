#include "HRC04.h"

/* DESCRIPCIO TODO */
//TRIGGER:  TX0->1
//ECHO:     21 SCL de la placa


HRC04 *sensDistancia;

void setup() {
  //Enabling serial communication
  Serial.begin(9600);
  sensDistancia = new HRC04();
}

void loop() {
  Serial.print(sensDistancia->getDistancia());
  _delay_ms(100); 
}

