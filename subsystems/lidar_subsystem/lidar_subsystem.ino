#include <Wire.h>

const int TFMini_ADDRESS = 0x10;  // I2C address for the TF-Luna
const int SLAVE_ADDRESS = 0x04;   // I2C address for this Arduino as a slave

const char PIN = 2;

const int MAX_DISTANCE = 225;

void setup() {
  Serial.begin(9600);
  pinMode(PIN, OUTPUT);
  Wire.begin(SLAVE_ADDRESS);  // Start I2C as slave
  Wire.begin();  // Join the I2C bus as a master
}

void loop() {
  Wire.beginTransmission(TFMini_ADDRESS);
  Wire.write(0x00);  // Start the measurement
  Wire.endTransmission();
  delay(100);  // Wait for measurement to complete

  Wire.requestFrom(TFMini_ADDRESS, 2);  // Request 2 bytes from TF-Luna
  if (Wire.available() == 2) {
    analogWrite(PIN, getDistance());

  }
  delay(1000);  // Delay before next measurement
}

uint16_t getDistance() {
  uint16_t distance = Wire.read() | Wire.read() << 8;

  if(distance > MAX_DISTANCE) 
    distance = MAX_DISTANCE;
    
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  return ((byte *)&distance, sizeof(distance));
}