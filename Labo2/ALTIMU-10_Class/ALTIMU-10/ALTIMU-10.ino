#include "ALTIMU-10.h"

/* Proyecto de arduino que es capaz de devolvernos
   la lectura del eje y del giroscopio del sensor
   ALTIMU-10 de Pololu */

ALTIMU10* gyro;

void setup() {

  gyro = new ALTIMU10();
  
}

void loop() {
  
  Serial.print("Inclinacion eje 'y' :");
  Serial.println(gyro->read_imu());
  
  _delay_ms(100);
  
}
