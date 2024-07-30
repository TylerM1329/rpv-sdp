#include <Wire.h>

void setup() {
  Wire.begin(8);
  Wire.onRequest(requestEvent);
}

void loop() {
  delay(100);
}

void requestEvent() {
  int x = 6;
  Wire.write(x);
}
