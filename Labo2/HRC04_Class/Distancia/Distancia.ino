#include "HRC04.h"

/*  Este programa utiliza las funcionalidades del sensor de distancia encapsuladas
    en una clase. Para ello, declaramos un objeto instancia de la clase y utilizamos sus
    métodos para simplificar el código, ya que las configuraciones y métodos estan 
    implementados dentro de la clase. Mostrar la distancia se reduce a 
    llamar un método en el loop que devuelve un valor y mostrarlo por serial. */

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

