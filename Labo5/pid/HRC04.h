#ifndef HRC04_H
#define HRC04_H

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

/* Clase destinada al uso del sensor
 * HRC04, es capaz de retornar la distancia
 * a la cual se encuentra un objecto de el */

class HRC04{
public:

	HRC04();
	~HRC04();
	double getDistancia();	
  
};

#endif
