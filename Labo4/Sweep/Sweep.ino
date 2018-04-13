#include <Servo.h>

#define FASTASFUCK 6
#define NOTSOFAST 13
#define SLOWASFUCK 28

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

void spin2Win(int speed){
  
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(/*TODO*/);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(/*TODO*/);                       // waits 15ms for the servo to reach the position
  }
}
void setup() {
  myservo.attach(9, 700, 2400);  // Custom min and max
}

void loop() {

  spin2Win(/*TODO*/); 
}

