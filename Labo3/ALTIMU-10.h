#ifndef ALTIMU10_H
#define ALTIMU10_H

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <LSM6.h>

/* Esta clase encapsula el uso
 * del sensor ALTIMU10, la funcion
 * read_imu() podria devolver una estructura
 * con todas las magnitudes que el sensor es 
 * capaz de leer, pero de momento, por simplicidad
 * tan solo retorna el valor del eje Y del gyro */

class ALTIMU10 {
	private:
		LSM6 imu;
		
	public:
		ALTIMU10();
		~ALTIMU10();
	
		double read_imu();	
};

#endif
