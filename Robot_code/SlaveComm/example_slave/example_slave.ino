#include "SlaveComm.h"

SlaveComm slave;

uint8_t masterMAC[] = {0xA4,0xCF,0x12,0xF5,0x20,0x53};

void setup() {
  Serial.begin(115200);

  slave.setID(0);
  slave.setMasterMACAddress(masterMAC);

  slave.begin("OPPO A53", "611b10a883c5");

  Serial.println("Robot listo");
}

void loop() {
  Serial.print("Ángulo: ");   Serial.println(slave.getAngle());
  Serial.print("Distancia: ");Serial.println(slave.getDistance());
  Serial.print("Out: ");      Serial.println(slave.getOut());
  delay(500);
}
