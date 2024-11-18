#include <Wire.h>
#include <SoftwareSerial.h>

const int TFL_TX_PIN = 10; // Connect this to TF-Luna RX pin
const int TFL_RX_PIN = 11; // Connect this to TF-Luna TX pin

SoftwareSerial tflSerial(TFL_TX_PIN, TFL_RX_PIN); // Create a SoftwareSerial port for TF-Luna

const int SLAVE_ADDRESS = 0x20; // I2C address for this Arduino
const int MAX_DISTANCE = 400;
uint16_t distance = 0;

uint8_t recvBuffer[9];
uint8_t recvIndex = 0;

void setup() {
  Wire.begin(SLAVE_ADDRESS);   // Start I2C as a slave
  Wire.onRequest(sendDistance); // Register function to send data on request
  Serial.begin(9600);          // Initialize serial communication for debugging
  tflSerial.begin(115200);     // Initialize UART for TF-Luna with 115200 baud rate
}

void loop() {
  readDistance();
}

// Function to read distance from TF-Luna via UART (non-blocking)
void readDistance() {
  while (tflSerial.available()) {
    uint8_t byte = tflSerial.read();

    if (recvIndex < 2 && byte == 0x59) {
      recvBuffer[recvIndex++] = byte;
    } else if (recvIndex >= 2) {
      recvBuffer[recvIndex++] = byte;
      if (recvIndex == 9) { // We have all 9 bytes
        uint16_t checksum = 0;
        for (int i = 0; i < 8; i++) {
          checksum += recvBuffer[i];
        }
        if ((checksum & 0xFF) == recvBuffer[8]) {
          distance = recvBuffer[2] | (recvBuffer[3] << 8);
          distance = (distance > MAX_DISTANCE || distance == 0) ? MAX_DISTANCE : distance;
          Serial.println(String(distance) + " cm");
        }
        recvIndex = 0; // Reset to start reading the next packet
      }
    } else {
      recvIndex = 0; // Reset if header bytes are incorrect
    }
  }
}

// Function to send the distance to the Raspberry Pi via I2C
void sendDistance() {
  Wire.write(distance & 0xFF);        // Send low byte of the distance
  Wire.write((distance >> 8) & 0xFF); // Send high byte of the distance
}
