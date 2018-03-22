#include "ALTIMU-10.h"

ALTIMU10* gyro;

void setup() {

  gyro = new ALTIMU10();
  
}

void loop() {
  
  Serial.print("Inclinacion eje 'y' :");
  Serial.println(gyro->read_imu());
  
  _delay_ms(100);
  
}
