#include <Wire.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <math.h>

SoftwareSerial gpsSerial(4, 3); // RX, TX
TinyGPSPlus gps;

const unsigned long timeoutMillis = 15000; // Extended Timeout for GPS initialization
unsigned long lastAttemptMillis = 0;

volatile double latitude = 0.0;
volatile double longitude = 0.0;
volatile double course = 0.0;

const int gpsResetPin = 7; // Pin to control GPS power/reset

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600); // Lower baud rate for more reliable SoftwareSerial handling
  Wire.begin(0x15); // I2C slave address
  Wire.onRequest(sendGPSData); // Set onRequest handler

  pinMode(gpsResetPin, OUTPUT);
  digitalWrite(gpsResetPin, HIGH); // Ensure GPS is powered on

  Serial.println("Initializing GPS...");
  
  if (!initializeGPS()) {
    Serial.println("Failed to initialize GPS. Reinitializing...");
    reinitializeGPS();
  }
}

void loop() {
  static double prevLatitude = 0.0;
  static double prevLongitude = 0.0;
  static double prevCourse = 0.0;

  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        double newLatitude = gps.location.lat();
        double newLongitude = gps.location.lng();

        if (newLatitude != 0.0 && newLongitude != 0.0 && (newLatitude != prevLatitude || newLongitude != prevLongitude)) {
          latitude = newLatitude;
          longitude = newLongitude;

          // Only calculate bearing if we have a previous valid point
          if (prevLatitude != 0.0 && prevLongitude != 0.0) {
            course = calculateBearing(prevLatitude, prevLongitude, latitude, longitude);
            if (course == 0.0) {
              // If bearing calculation fails, use previous course
              course = prevCourse;
            } else {
              prevCourse = course;
            }
          } else {
            // If no previous point, retain the previous course value
            course = prevCourse;
          }

          // Update previous latitude and longitude for the next calculation
          prevLatitude = latitude;
          prevLongitude = longitude;

          Serial.print("Calculated Course Angle: ");
          Serial.print(course);
          Serial.print("     Latitude: ");
          Serial.print(latitude, 6);
          Serial.print("     Longitude: ");
          Serial.println(longitude, 6);

          // Update the last attempt time since new data has been processed
          lastAttemptMillis = millis();
        } else {
          Serial.println("Waiting for valid location...");
          Serial.print("Calculated Course Angle: ");
          Serial.print(course);
          Serial.print("     Latitude: ");
          Serial.print(latitude, 6);
          Serial.print("     Longitude: ");
          Serial.println(longitude, 6);
        }
      }
    }
  }

  // Check if GPS is unresponsive and restart if necessary
  if (millis() - lastAttemptMillis > timeoutMillis) {
    Serial.println("GPS timeout. Resetting GPS...");
    resetGPS();
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
  Wire.write((byte*)&course_scaled, sizeof(course_scaled));  // Sending course over I2C // 2 bytes
}

// Function to reset GPS module
void resetGPS() {
  digitalWrite(gpsResetPin, LOW); // Turn off GPS module
  delay(1000); // Wait for a second
  digitalWrite(gpsResetPin, HIGH); // Turn GPS module back on
  delay(2000); // Wait for GPS to stabilize
}

// Function to calculate bearing (course) in degrees
double calculateBearing(double lat1, double lon1, double lat2, double lon2) {
  // Convert latitude and longitude from degrees to radians
  lat1 = radians(lat1);
  lon1 = radians(lon1);
  lat2 = radians(lat2);
  lon2 = radians(lon2);

  // Calculate the difference in longitude
  double dLon = lon2 - lon1;

  // Calculate components for atan2
  double y = sin(dLon) * cos(lat2);
  double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);

  // Calculate the bearing in radians
  double bearing = atan2(y, x);

  // Convert the bearing from radians to degrees
  bearing = degrees(bearing);

  // Normalize the bearing to be within 0-360 degrees
  bearing = fmod((bearing + 360), 360);

  return bearing; // Return the bearing in degrees
}
