#include <NMEAGPS.h>
#include <SoftwareSerial.h>
#include <Wire.h>

static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;

// Set up the software serial port
SoftwareSerial gpsSerial(RXPin, TXPin);

NMEAGPS gps;
gps_fix currentFix; // Stores the current GPS coordinates

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(GPSBaud);

  Wire.begin(0x15); // I2C slave address
  Wire.onRequest(sendGPSData); // Set onRequest handler

  Serial.println("Initializing GPS...");
}

void loop() {
  if (gps.available(gpsSerial)) {
    currentFix = gps.read(); // Read new GPS data

      Serial.print(F("Location: "));
      Serial.print(currentFix.latitude(), 6);
      Serial.print(F(", "));
      Serial.print(currentFix.longitude(), 6);
      Serial.print(F(", Course: "));
      Serial.print(currentFix.heading(), 2);
      Serial.println(F(" deg"));
  }
}

// Function to send GPS data when requested
void sendGPSData() {
  if (currentFix.valid.location) {
    float latitude = currentFix.latitude();
    float longitude = currentFix.longitude();
    int16_t heading_scaled = (int16_t)(currentFix.heading() * 100); // Scale by 100 to keep precision

    Wire.write((byte*)&latitude, sizeof(latitude));  // Send latitude
    Wire.write((byte*)&longitude, sizeof(longitude));  // Send longitude
    Wire.write((byte*)&heading_scaled, sizeof(heading_scaled));  // Send heading
  }
}
