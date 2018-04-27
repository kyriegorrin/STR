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
 
Servo myservo;  // create servo object to control a servo

HRC04* dist_sensor;

//Function to map distance from the sun and beyond and map it to the servo
void distanceToPosition(){

  int mm = 0;
  for(int i = 0; i < 3; ++i){
    mm += (int) dist_sensor->getDistancia();
  } mm = mm/3;
  
  Serial.println(mm);

  //We saturate the value to 180 so we don't destroy the servo
  if(mm > 180) mm = 180;
  myservo.write(mm);  
}

void setup() {
  Serial.begin(9600);
  myservo.attach(9, 700, 2400);  // Custom min and max
  //Creating object for the sensor
  dist_sensor = new HRC04();
}

void loop() {
  distanceToPosition();
}

