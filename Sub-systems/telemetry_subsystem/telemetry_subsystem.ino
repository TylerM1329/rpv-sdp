#include <NMEAGPS.h>
#include <SoftwareSerial.h>
#include <Wire.h>

// Pin definitions for GPS module
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;

// Software serial for GPS communication
SoftwareSerial gpsSerial(RXPin, TXPin);

// NMEA GPS parser and data storage
NMEAGPS gps;
gps_fix currentFix; // Stores the latest GPS data

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(GPSBaud);

  Wire.begin(0x15); // Set I2C slave address
  Wire.onRequest(sendGPSData); // Handler to send GPS data on request

  Serial.println("Initializing GPS...");
  
  // Developer Note:
  // 1. The course angle (heading) reported by the GPS module might not always be accurate.
  //    It is typically valid only when the device is moving at a sufficient speed.
  // 2. Ensure the GPS has a fix with at least 4 satellites for reliable location data.
}

void loop() {
  // Check for new GPS data
  if (gps.available(gpsSerial)) {
    currentFix = gps.read(); // Read the latest GPS fix

    // Output GPS data to the serial monitor
    Serial.print(F("Location: "));
    Serial.print(currentFix.latitude(), 6);
    Serial.print(F(", "));
    Serial.print(currentFix.longitude(), 6);
    Serial.print(F(", Course: "));
    Serial.print(currentFix.heading(), 2);
    Serial.println(F(" deg"));
    
    // Developer Note:
    // Use `currentFix.satellites` to check the number of satellites if needed for diagnostics.
    // Example:
    // Serial.print(F("Satellites: "));
    // Serial.println(currentFix.satellites);
  }
}

// Send GPS data via I2C when requested
void sendGPSData() {
  if (currentFix.valid.location) {
    float latitude = currentFix.latitude();
    float longitude = currentFix.longitude();
    int16_t heading_scaled = (int16_t)(currentFix.heading() * 100); // Scale heading for precision

    // Send data via I2C
    Wire.write((byte*)&latitude, sizeof(latitude));   // Latitude
    Wire.write((byte*)&longitude, sizeof(longitude)); // Longitude
    Wire.write((byte*)&heading_scaled, sizeof(heading_scaled)); // Heading
  }
}
