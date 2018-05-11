#include <Servo.h>
#include "HRC04.h"

//------------------------------------ARTEMANÍACOS SECTION-----------------------------//
/*
 * El código es simple y no hay mucho que explicar. Pero lo poco que hay, lo explicamos.
 * Utilizando el sensor de distancia (y nuestra librería) obtenemos un valor en milímetros,
 * el cual usaremos para mover el servo hasta un máximo de 180º.
 * 
 * Saturamos el valor a 180º ya que es el valor máximo que permite el servo y no queremos
 * destruirlo, con los daños y perjuicios que pueda suponer.
 * 
 * Para estabilizar un poco los movimientos, hacemos medias aritméticas de cada 3 entradas
 * de distancia. También mostramos por serial el valor leído del sensor, por si es necesario
 * comprobarlos y porque queda bonito.
 */
//-------------------------------------------------------------------------------------//

//GRAU MIG  PER LINIA RECTA: 84

Servo myservo;  // create servo object to control a servo

HRC04* dist_sensor;

//Variable del PID
unsigned long lastTime;
double Input, Output, Setpoint, Degree;
double errSum, lastErr;
double kp, ki, kd;

//Function to map distance from the sun and beyond and map it to the servo
void distanceToPosition(){

  int mm = 0;
  for(int i = 0; i < 4; ++i){
    mm += (int) dist_sensor->getDistancia();
  } mm = mm/4;

  //We saturate the value to 180 so we don't destroy the servo
  if(mm > 180) mm = 180;
  myservo.write(mm);  
}

//Function to get the mean of 4 measures in milimeters
double getMeanDistance(){
  double mm = 0;
  for(int i = 0; i < 4; ++i){
    mm += dist_sensor->getDistancia();
  } mm = mm/4;
  
  return mm;
}

//PID MEMES
void Compute(){
  /*How long since we last calculated*/
  unsigned long now = millis();
  double timeChange = (double)(now - lastTime);
  
  /*Compute all the working error variables*/
  double error = Setpoint - Input;
  errSum += (error * timeChange);
  double dErr = (error - lastErr) / timeChange;
  
  /*Compute PID Output*/
  Output = kp * error + ki * errSum + kd * dErr;
  
  /*Remember some variables for next time*/
  lastErr = error;
  lastTime = now;
}

//PID MEME-TUNING
void SetTunings(double Kp, double Ki, double Kd){
  kp = Kp;
  ki = Ki;
  kd = Kd;
}

void setup() {
  Serial.begin(9600);
  myservo.attach(9, 700, 2400);  // Custom min and max
  
  //Creating object for the sensor
  dist_sensor = new HRC04();

  //Tunejant el Eight-Six
  SetTunings(0.9, 0 , 0.5);

  //Quin set point volem en mm?
  Setpoint = 100;
}

void loop() {
  //distanceToPosition();
  //myservo.write(90);

  Input = getMeanDistance();
  Compute();
  
  if (Output <= -200) Output = -200;
  else if (Output >= 100) Output = 100;
  
  Degree = map(Output, -200, 100, 84-20, 84+20);
  myservo.write(Degree);
  Serial.print(Input);
  Serial.print("   ");
  Serial.println(Output);
}

