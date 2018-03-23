#include <stdlib.h>
#include <stdio.h>
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include "HRC04.h"

byte myMAC[] = {0x90, 0xA2, 0xDA, 0x0F, 0x4A, 0x03};
IPAddress myIP(192, 168, 1, 3);
IPAddress destIP(192, 168, 1, 8);//Soyboy destination
unsigned int localPort = 8888;
char sendBuffer[5];
EthernetUDP Udp;
double dist;

HRC04 *sensDistancia;

void setup(){
  
  Ethernet.begin(myMAC, myIP);
  Udp.begin(localPort);
  Serial.begin(9600);
  sensDistancia = new HRC04();
}

void loop(){
  dist = sensDistancia->getDistancia();
  itoa((float)dist, sendBuffer, 10);
  //sprintf(sendBuffer, sizeof(sendBuffer), "%f", dist);
  Udp.beginPacket(destIP, 8889);
  Udp.write(sendBuffer);
  Udp.endPacket();
  //Neteja sendbuffer
  //sendBuffer[0] = '\0';
  delay(1000);
}
