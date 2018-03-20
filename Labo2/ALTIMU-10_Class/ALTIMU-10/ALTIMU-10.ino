#include "ALTIMU-10.h"

typedef ALTIMU10 sensor;

sensor gyro;

void setup() {
  Serial.begin(9600);
}

void loop() {
  gyro.read_imu();
}
