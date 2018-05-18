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

//GRAU MIG  PER LINIA RECTA: 82-84

//HRC PINS: TRIGGER:1, ECHO: 19

Servo myservo;  // create servo object to control a servo

//PULSEIN variables
int trigPin = 11;    //Trig - green Jumper
int echoPin = 12;    //Echo - yellow Jumper
unsigned long duration, cm, inches;
double mm = 0;

//Variable del PID
unsigned long lastTime;
double Input, Output, Setpoint, Degree;
double errSum, lastErr;
double kp, ki, kd;

double lastInput;

/*
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
*/

//Function to get distance pulsein
double getDistance(){

  //Puta per assegurar-se de una bona lectura
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  //Lectura pulsein
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);

  //Conversions de distancia
  return (duration/2) / 2.91;
}

//Getting the mean of some distances
double getMeanDistance(){
  mm = 0;
  for(int i = 0; i < 2; ++i){
    mm += getDistance();
  } mm = mm/2;

  return mm;
}

//Funció de filtratge de inputs (assumim que diferencies elevades entre
//inputs és un pic estrany i l'aplanem com podem)
void filterInput(){
  double diff = Input - lastInput;
  double diffAbs = abs(diff);
  if (diffAbs >= 50){
    Input = 0.4*(diff) + lastInput;
  }
  lastInput = Input;
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
  //dist_sensor = new HRC04();

  //Pinmodes pulsein
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  //Tunejant el Eight-Six
  SetTunings(0.2, 0 , 10);

  //Quin set point volem en mm?
  Setpoint = 160;

  lastInput = getMeanDistance();
}

void loop() {
  //Lectura i quimioterapia de input
  Input = getMeanDistance();
  filterInput();

  //Saturació valors input (Distancies superiors a 280 el tornen boig)
  if(Input > 300) Input = 300;
  
  //Computas y a lo loco
  Compute();

  //Saturació de valors de output per si el sensor es torna excessivament boig
  if (Output <= -200) Output = -200;
  else if (Output >= 200) Output = 200;

  //Mapejar rangs de graus i moure
  Degree = map(Output, -200, 200, 84-30, 84+30);
  myservo.write(Degree);
  
  //Takumi installed a new tachometer and now can check his RPMs
  //Si no poses aquests prints, el servo no fa write WTF QUE
  Serial.print(Input);
  Serial.print("   ");
  Serial.println(Output);
}

