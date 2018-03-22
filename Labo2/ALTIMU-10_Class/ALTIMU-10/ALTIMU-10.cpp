#include "ALTIMU-10.h"
#include <Wire.h>


ALTIMU10::ALTIMU10() {
  Serial.begin(9600);
  Wire.begin();

  if (!imu.init())
  {
    Serial.println("La abuela ya estaba muerta");
    while (1);
  }
	imu.enableDefault();
  
}

ALTIMU10::~ALTIMU10(){/*L'avia morta que la tracti algu altre*/}

double ALTIMU10::read_imu() {
	
	imu.read(); //Lo que se lea del sensor va a parar a "imu"...

  return RAD_TO_DEG * (atan2(-imu.g.x, -imu.g.z)+PI);
}

