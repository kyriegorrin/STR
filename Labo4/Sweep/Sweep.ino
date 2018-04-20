#include <Servo.h>

#define FASTASFUCK 30
#define NOTSOFAST 15
#define SLOWASFUCK 5

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards


//Funció que fa rotació a tants Degree Per Second com indiquem 
void spin2Win(int dps){
  
  int retraso = (int) 1000.0/dps; 
  int pos = 0;    // variable to store the servo position

  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(retraso);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(retraso);                       // waits 15ms for the servo to reach the position
  }
}

void setup() {
  Serial.begin(9600);
  myservo.attach(9, 700, 2400);  // Custom min and max
}

void loop() {
  spin2Win(30);
}

