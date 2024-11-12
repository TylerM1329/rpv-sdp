#include <Wire.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

SoftwareSerial gpsSerial(4, 3); // RX, TX
TinyGPSPlus gps;

const unsigned long timeoutMillis = 15000; // Extended Timeout for GPS initialization
unsigned long lastAttemptMillis = 0;

volatile double latitude = 0.0;
volatile double longitude = 0.0;
volatile double course = 0.0;

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600); // Lower baud rate for more reliable SoftwareSerial handling
  Wire.begin(0X15); // I2C slave address
  Wire.onRequest(sendGPSData); // Set onRequest handler

  Serial.println("Initializing GPS...");
  
  if (!initializeGPS()) {
    Serial.println("Failed to initialize GPS. Reinitializing...");
    reinitializeGPS();
  }
}

void loop() {
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        if (gps.location.lat() != 0.0) { latitude = gps.location.lat(); }
        if (gps.location.lng() != 0.0) { longitude = gps.location.lng(); }
        if (gps.course.isValid()) { course = gps.course.deg(); } // Check validity before assigning

        Serial.print("Course Angle: ");
        Serial.print(course);
        Serial.print("     Latitude: ");
        Serial.print(latitude, 6);
        Serial.print("     Longitude: ");
        Serial.println(longitude, 6);

        // Update the last attempt time since new data has been processed
        lastAttemptMillis = millis();
      } else {
        Serial.println("Waiting for valid location...");
      }
    }
  }

  // Check if GPS is unresponsive and restart if necessary
  if (millis() - lastAttemptMillis > timeoutMillis) {
    Serial.println("GPS timeout. Reinitializing...");
    reinitializeGPS();
  }
}

// Function to reinitialize GPS instead of restarting Arduino
void reinitializeGPS() {
  gpsSerial.end();
  gpsSerial.begin(4800); // Reinitialize SoftwareSerial
  lastAttemptMillis = millis(); // Reset attempt timer
}

// Function to initialize GPS
bool initializeGPS() {
  unsigned long startMillis = millis();
  while (millis() - startMillis < timeoutMillis) {
    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
      if (gps.location.isValid()) {
        Serial.println("GPS initialized successfully.");
        lastAttemptMillis = millis();
        return true;
      }
    }
  }
  return false; // Initialization failed
}

// Function to send GPS data when requested
void sendGPSData() {
  // Scaling and sending course (angle) in degrees
  int course_scaled = course * 100;  // Scale by 100 to keep two decimal places
  
  Wire.write((byte*)&latitude, sizeof(latitude)); // Send latitude // 4 bytes
  Wire.write((byte*)&longitude, sizeof(longitude)); // Send longitude // 4 bytes
  Wire.write((uint8_t*)&course_scaled, sizeof(course_scaled));  // Sending course over I2C // 2 bytes
}
