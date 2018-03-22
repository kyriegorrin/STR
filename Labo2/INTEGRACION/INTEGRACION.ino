#include  "HRC04.h"
#include  "ALTIMU-10.h"

/* Este proyecto integra los dos objetos en
   un solo fichero, nos permite ver por el serial
   plotter (imprime lo que le llega por el serial)
   los valores leidos por ambos sensores */

void setup() {
	ALTIMU10* imu;
	HRC04*	  ultrasonido;

	imu = new ALTIMU10();
	ultrasonido = new HRC04();

	Serial.begin(9600);
}

void loop() {
	Serial.print(sensDistancia->getDistancia());
	Serial.print('\t');
	Serial.println(imu->read_imu()); //El orden en el que hemos hecho los prints (con el tab en medio)
									 //Permite que el plotter imprima los dos valores constantemente 

	__delay_ms(100);
}


