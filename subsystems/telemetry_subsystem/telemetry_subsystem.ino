#include <NMEAGPS.h>
#include <SoftwareSerial.h>
#include <Wire.h>

static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;

// Set up the software serial port
SoftwareSerial gpsSerial(RXPin, TXPin);

NMEAGPS gps;
gps_fix fix;

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(GPSBaud);
  
  Wire.begin(0x15); // I2C slave address
  Wire.onRequest(sendGPSData); // Set onRequest handler

  Serial.println("Initializing GPS...");
}

void loop() {
  if (gps.available(gpsSerial)) {
    fix = gps.read();  // Read the GPS data

    // Output latitude, longitude, and course if valid
    if (fix.valid.location) {
      Serial.print(F("Location: "));
      Serial.print(fix.latitude(), 6);
      Serial.print(F(", "));
      Serial.print(fix.longitude(), 6);

      if (fix.valid.heading) {
        Serial.print(F(", Course Angle: "));
        Serial.print(fix.heading());
        Serial.println(F(" deg"));
      } else {
        Serial.println(F(", Course Angle: invalid"));
      }
    }
  }
}

// Function to send GPS data when requested
void sendGPSData() {
  if (fix.valid.location) {
    float latitude = fix.latitude();
    float longitude = fix.longitude();
    int16_t course_scaled = (fix.valid.heading ? (int16_t)(fix.heading() * 100) : 0);  // Scale by 100 to keep precision, -1 if invalid

    Wire.write((byte*)&latitude, sizeof(latitude));  // Send latitude
    Wire.write((byte*)&longitude, sizeof(longitude));  // Send longitude
    Wire.write((byte*)&course_scaled, sizeof(course_scaled));  // Send course
  }
}
