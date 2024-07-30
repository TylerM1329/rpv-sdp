#include <Wire.h>

int rxVal = 0;

void setup() {
  Wire.begin(4);
  Wire.onReceive(receiveEvent);
  Serial.begin(9600);
}

void loop() {
  Serial.println(rxVal);
  delay(100);
}

void receiveEvent() {
  rxVal = Wire.read();
}