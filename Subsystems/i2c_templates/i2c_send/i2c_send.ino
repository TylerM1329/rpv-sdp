#include <Wire.h>

char txBuf[3] = {'3', '3', '3'};

int x = 5;
void setup() {
  Wire.begin(8);
}

void loop() {
  Wire.beginTransmission(4);
  Wire.write(x);
  Wire.endTransmission();
  delay(1000);
}

