#include <Encoder.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include <stdio.h>
#include <string.h>

// Encoder Setup
const int encoderPinA = 2;
const int encoderPinB = 3;

const double pulsesPerRevolution = 600.0;  // Encoder Pulses per Revolution
const double gearRatio = 16.0;             // Gear Ratio between wheel gear and encoder gear

double rpm = 0;

Encoder myEncoder(encoderPinA, encoderPinB);

long prevPosition = 0;
unsigned long prevTime = 0;

// GPS Setup
static const int RXPin = 4;
static const int TXPin = 5;
static const uint32_t GPSBaud = 9600;
float lat, lon;
double course;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial gpsSerial(RXPin, TXPin);

void setup() 
{
  Serial.begin(9600);
  Wire.begin(0X15);  // Set I2C address for this device
  Wire.onRequest(requestEvent);  // Register function to send data on I2C request
  gpsSerial.begin(GPSBaud);  // Start the serial communication with the GPS module
}

void loop() 
{
  /*
  // Rotary Encoder Code
  long currentPosition = myEncoder.read();  // Reading Encoder Position
  unsigned long currentTime = millis();     // Reading Current Time

  // Calculating Position Change
  long positionChange = currentPosition - prevPosition;

  // Calculating Time Change
  unsigned long timeChange = currentTime - prevTime;

  if (timeChange > 0)
  {
    // Calculating Wheel Speed
    double wheelSpeed = static_cast<double>(positionChange) / timeChange;

    // Convert to RPM with the gear ratio
    rpm = (wheelSpeed * 60.0 * 1000.0) / (pulsesPerRevolution * gearRatio);

    Serial.print("Wheel RPM: ");
    Serial.println(rpm);

    prevPosition = currentPosition;
    prevTime = currentTime;
  }
  */

  // GPS Code
  while (gpsSerial.available() > 0)
  {
    if (gps.encode(gpsSerial.read()))
      displayInfo();
  }

  // If 5000 milliseconds pass and there are no characters coming in
  // over the software serial port, show a "No GPS detected" error
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println("No GPS detected");
    while (true);  // Stop the loop if no GPS is detected
  }

  delay(100);  // Adjust the delay based on your requirements
}

// Function to display GPS data (latitude, longitude, course)
void displayInfo()
{
  // Latitude
  lat = gps.location.lat();
  Serial.print("Latitude: ");
  Serial.println(lat, 6);  // Print latitude with 6 decimal places

  // Longitude
  lon = gps.location.lng();
  Serial.print("Longitude: ");
  Serial.println(lon, 6);  // Print longitude with 6 decimal places

  // Course (angle/direction in degrees)
  course = gps.course.deg();
  Serial.print("Course (angle): ");
  Serial.println(course, 2);  // Print course with 2 decimal places

  Serial.println();
  delay(100);  // Adjust the delay based on your needs
}

// I2C function to send GPS data to Raspberry Pi
void requestEvent()
{
  long lat_scaled = abs(lat) * 1000000;               // Scaling Latitude to an int
  long lon_scaled = abs(lon) * 1000000;               // Scaling Longitude to an int
  Wire.write((uint8_t*)&lat_scaled, sizeof(lat_scaled));  // Sending scaled Latitude over I2C
  Wire.write((uint8_t*)&lon_scaled, sizeof(lon_scaled));  // Sending scaled Longitude over I2C

  // Scaling and sending course (angle) in degrees
  int course_scaled = course * 100;  // Scale by 100 to keep two decimal places
  Wire.write((uint8_t*)&course_scaled, sizeof(course_scaled));  // Sending course over I2C

  // Checking Latitude and Longitude for negative values to send control bit
  if (lat > 0 && lon > 0)                             // Case 0 if both Lat and Long are positive
  {
    int ctrl = 0;                                     
    Wire.write((uint8_t*)&ctrl, 1);                   // Send control bit to Raspberry Pi through I2C
  }
  else if (lat > 0 && lon < 0)                        // Case 1 if Lat is positive and Long is negative
  {
    int ctrl = 1;                                     
    Wire.write((uint8_t*)&ctrl, 1);                   // Send control bit to Raspberry Pi through I2C
  }
  else if (lat < 0 && lon > 0)                        // Case 2 if Lat is negative and Long is positive
  {
    int ctrl = 2;                                     
    Wire.write((uint8_t*)&ctrl, 1);                   // Send control bit to Raspberry Pi through I2C
  }
  else if (lat < 0 && lon < 0)                        // Case 3 if Lat is negative and Long is negative
  {
    int ctrl = 3;                                     
    Wire.write((uint8_t*)&ctrl, 1);                   // Send control bit to Raspberry Pi through I2C
  }

  Serial.println("Sent GPS Data to Raspberry Pi");
}