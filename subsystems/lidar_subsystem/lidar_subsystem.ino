#include <Wire.h>
#include <SoftwareSerial.h>
#include <TFLI2C.h>

const int TFL_TX_PIN = 10; // Connect this to TF-Luna RX pin
const int TFL_RX_PIN = 11; // Connect this to TF-Luna TX pin

SoftwareSerial tflSerial(TFL_TX_PIN, TFL_RX_PIN); // Create a SoftwareSerial port for TF-Luna

const int SLAVE_ADDRESS = 0x20; // I2C address for this Arduino
const int MAX_DISTANCE = 400;
uint16_t distance = 0;

// Function prototypes
uint8_t getLowByte(uint16_t value);
uint8_t getHighByte(uint16_t value);
void sendDistance();

void setup() {
  Serial.println(" ASS");
  Wire.begin(SLAVE_ADDRESS);   // Start I2C as a slave
  Wire.onRequest(sendDistance); // Register function to send data on request
  Serial.begin(9600);          // Initialize serial communication for debugging
  tflSerial.begin(115200);     // Initialize UART for TF-Luna with 115200 baud rate
}

void loop() {
  if (readDistance()) {
    Serial.println(String(distance) + " cm");
  }

  delay(50); // Delay before next measurement
}

// Function to read distance from TF-Luna via UART
bool readDistance() {
  uint8_t recvBuffer[9];

  if (tflSerial.available()) {
    // Read the data packet (TF-Luna sends 9 bytes per packet)
    if (tflSerial.readBytes(recvBuffer, 9) == 9) {
      // Validate the data packet
      if (recvBuffer[0] == 0x59 && recvBuffer[1] == 0x59) {
        // Calculate checksum
        uint16_t checksum = 0;
        for (int i = 0; i < 8; i++) {
          checksum += recvBuffer[i];
        }
        checksum &= 0xFF; // Keep it to 8 bits

        // Check if checksum matches
        if (checksum == recvBuffer[8]) {
          // Extract the distance value (low byte first, then high byte)
          distance = recvBuffer[2] | (recvBuffer[3] << 8);

          // Optional: limit distance to MAX_DISTANCE to prevent outliers
          if (distance > MAX_DISTANCE || distance == 0) {
            distance = MAX_DISTANCE; // Cap the value if it exceeds MAX_DISTANCE
          }

          return true;
        }
      }
    }
  }
  return false;
}


// Function to send the distance to the Raspberry Pi via I2C
void sendDistance() {
  uint8_t lowByte = getLowByte(distance);
  uint8_t highByte = getHighByte(distance);

  Wire.write(lowByte);  // Send low byte of the distance
  Wire.write(highByte); // Send high byte of the distance
}

// Helper function to extract low byte from a 16-bit value
uint8_t getLowByte(uint16_t value) {
  return value & 0xFF;
}

// Helper function to extract high byte from a 16-bit value
uint8_t getHighByte(uint16_t value) {
  return (value >> 8) & 0xFF;
}
