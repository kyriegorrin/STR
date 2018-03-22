#include "ALTIMU-10.h"
#include <Wire.h>

/* Pretende que el metodo read_imu() nos retorne
 * la inclinacion en el eje Y... (que no es lo mismo
 * que el eje Y del gyro, que se expresa en dps) */

ALTIMU10::ALTIMU10() {
  Serial.begin(9600);
  Wire.begin();

  if (!imu.init())
  {
    Serial.println("No se puede establecer al comunicacion con el sensor...");
    while (1);
  }
	imu.enableDefault();
  
}

ALTIMU10::~ALTIMU10(){}

double ALTIMU10::read_imu() {
	
	imu.read(); //Lo que se lea del sensor va a parar a "imu"...

  return RAD_TO_DEG * (atan2(-imu.g.x, -imu.g.z)+PI); //Se intentan devolver los grados...
}

