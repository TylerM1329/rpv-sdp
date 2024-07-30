#include <Wire.h>

// the pi will do the same thing as what this program does.
// it will request data from a subsystem and parse the reply.

void setup() {
  Wire.begin();
  Serial.begin(115200);
}

void loop() {
  Wire.requestFrom(8, 1);

  while(Wire.available()) {
    char c = Wire.read();
    Serial.println(c, DEC);
  }
  delay(10);
}
