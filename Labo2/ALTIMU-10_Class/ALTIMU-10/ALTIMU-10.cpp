#include "ALTIMU-10.h"

ALTIMU10::ALTIMU10() {
	
	Serial.begin(9600);
  	Wire.begin();

	imu.enableDefault();
}

ALTIMU10::~ALTIMU10(){/*L'avia morta que la tracti algu altre*/}

void ALTIMU10::read_imu() {
	imu.read(); //Lo que se lea del sensor va a parar a "imu"...

	Serial.print("Inclinacion eje 'y' :");
	Serial.println(imu.g.y);

	_delay_ms(100);
}

