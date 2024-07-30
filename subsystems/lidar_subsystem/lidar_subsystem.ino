#include <Wire.h>

const int TFMini_ADDRESS = 0x10;  // I2C address for the TF-Luna
const int SLAVE_ADDRESS = 0x04;   // I2C address for this Arduino as a slave

void setup() {
  Serial.begin(9600);
  Wire.begin(SLAVE_ADDRESS);  // Start I2C as slave
  Wire.onRequest(requestEvent);  // Register event for master's request
  Wire.begin();  // Join the I2C bus as a master
}

void loop() {
  Wire.beginTransmission(TFMini_ADDRESS);
  Wire.write(0x00);  // Start the measurement
  Wire.endTransmission();
  delay(100);  // Wait for measurement to complete

  Wire.requestFrom(TFMini_ADDRESS, 2);  // Request 2 bytes from TF-Luna
  if (Wire.available() == 2) {
    uint16_t distance = Wire.read() | Wire.read() << 8;
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
  }
  delay(1000);  // Delay before next measurement
}

void requestEvent() {
  uint16_t distance = getDistance();  // Assuming getDistance() fetches the latest distance
  Wire.write((byte *)&distance, sizeof(distance));  // Send distance as two bytes
}

uint16_t getDistance() {
  // Insert code here that fetches and returns the latest distance measured
  return 0;  // Placeholder
}