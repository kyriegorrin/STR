#include <Servo.h>
#include "HRC04.h"

#define FASTASFUCK 120
#define NOTSOFAST 60
#define SLOWASFUCK 15

//TODO: EXPLAIN CODE AND CALCULATIONS

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

HRC04* dist_sensor;


//Funció que fa rotació a tants Degree Per Second com indiquem 
void spin2Win(int dps){
  
  int retraso = 1000/dps; 
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

//Function to map distance from the sun and beyond and map it to the servo
void distanceToPosition(){

  int mm = dist_sensor->getDistancia();;
  /*for(int i = 0; i < 5; ++i){
    mm += dist_sensor->getDistancia();
  }*/
  
  mm = mm/5;
  Serial.println(mm);

  if(mm > 180) mm = 180;
  myservo.write(mm);  
}

void setup() {
  Serial.begin(9600);
  myservo.attach(9, 700, 2400);  // Custom min and max

  dist_sensor = new HRC04();
  
}

void loop() {
  //spin2Win(180);
  distanceToPosition();
}

