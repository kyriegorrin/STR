#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include "HRC04.h"
#include "ALTIMU-10.h"

byte myMAC[] = {0x90, 0xA2, 0xDA, 0x0F, 0x4A, 0x03};
IPAddress myIP(192, 168, 1, 3);
IPAddress destIP(192, 168, 1, 8);//Soyboy destination
unsigned int localPort = 8888;
char sendBuffer[5];
EthernetUDP Udp;

int dist;
int deg;

HRC04*  sensDistancia;
ALTIMU10* gyro;

void setup(){
  
  Ethernet.begin(myMAC, myIP);
  Udp.begin(localPort);
  Serial.begin(9600);
  sensDistancia = new HRC04();
  gyro = new ALTIMU10();
}

void loop(){
  dist = sensDistancia->getDistancia();
  deg  = (int) gyro->read_imu();
  /*
  itoa(dist, sendBuffer, 10);
  Udp.beginPacket(destIP, 8889);
  Udp.write(sendBuffer);
  Udp.endPacket();
  
  Udp.beginPacket(destIP, 8889);
  itoa(deg, sendBuffer, 10);
  Udp.write(sendBuffer);
  Udp.endPacket();*/

  //New soplanucas code unified version
  Udp.beginPacket(destIP, 8889);
  itoa(dist, sendBuffer, 10);
  Udp.write(sendBuffer);
  itoa(deg, sendBuffer, 10);
  Udp.write(",");
  Udp.write(sendBuffer);
  Udp.endPacket();

  //Muerdealmohadas code here
  //TODO

  //Debug purposes (it's plottable)
  Serial.print(dist);
  Serial.print(" ");
  Serial.println(deg);
  
  delay(1000);
}
