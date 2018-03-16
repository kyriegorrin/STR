#ifndef LED_H
#define LED_H

#include <Arduino.h>

class LED { 
public:
	LED();
	~LED();
  void on();
	void off();
	void blink(int time);
};

#endif
