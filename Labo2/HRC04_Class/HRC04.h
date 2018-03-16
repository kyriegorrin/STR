#ifndef HRC04_H
#define HRC04_H

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

class HRC04{
public:
	HRC04();
	~HRC04();
	double getDistancia();	
}
