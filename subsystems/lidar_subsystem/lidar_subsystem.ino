#include <Wire.h>

const int TFMini_ADDRESS = 0x10;  // I2C address for the TF-Luna (sensor)
const int SLAVE_ADDRESS = 0x11;   // I2C address for this Arduino
const int MAX_DISTANCE = 400;
uint16_t distance = 0;

// Function prototypes
uint8_t getLowByte(uint16_t value);
uint8_t getHighByte(uint16_t value);
void sendDistance();

void setup() {
  Wire.begin(SLAVE_ADDRESS);  // Start I2C as a slave
  Wire.onRequest(sendDistance);  // Register function to send data on request
  Serial.begin(9600);  // Initialize serial communication for debugging
}

void loop() {
  // Simulate reading distance from the Lidar sensor (replace with actual sensor code)
  Wire.beginTransmission(TFMini_ADDRESS);
  Wire.write(0x00);  // Start the measurement (for the actual sensor)
  Wire.endTransmission();
  delay(100);  // Wait for measurement to complete

  Wire.requestFrom(TFMini_ADDRESS, 2);  // Request 2 bytes from TF-Luna
  if (Wire.available() == 2) {
    uint8_t lowByte = Wire.read();
    uint8_t highByte = Wire.read();
    distance = (highByte << 8) | lowByte;  // Combine bytes into 16-bit value

  // todo dont let it go to 0 bc no light reflects on nothin' - see how this plays out
    if (distance > MAX_DISTANCE) {
      distance = MAX_DISTANCE;  // Cap the distance at 225 cm
    }
    if (distance == 0)
    {
      distance = MAX_DISTANCE;
    }

    // Print distance for debugging purposes
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.print(" cm ");
    Serial.print("Sending low byte: ");
    Serial.print(lowByte, HEX);
    Serial.print("  Sending high byte: ");
    Serial.println(highByte, HEX);
  }

  delay(500);  // Delay before next measurement
}

// Function to send the distance to the Raspberry Pi via I2C
void sendDistance() {
  //Serial.println("distance set");
  uint8_t lowByte = getLowByte(distance);
  uint8_t highByte = getHighByte(distance);

  Wire.write(lowByte);  // Send low byte of the distance
  Wire.write(highByte);  // Send high byte of the distance
}

// Helper function to extract low byte from a 16-bit value
uint8_t getLowByte(uint16_t value) {
  return value & 0xFF;
}

// Helper function to extract high byte from a 16-bit value
uint8_t getHighByte(uint16_t value) {
  return (value >> 8) & 0xFF;
}
