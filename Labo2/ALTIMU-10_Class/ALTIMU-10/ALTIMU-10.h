#ifndef ALTIMU10_H
#define ALTIMU10_H

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <Wire.h>
#include <LSM6.h>

class ALTIMU10 {
	private:
		LSM6 imu;
		
	public:
		ALTIMU10();
		~ALTIMU10();
	
		void read_imu();	
};

#endif
