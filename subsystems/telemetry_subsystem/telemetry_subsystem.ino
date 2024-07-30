#include <Encoder.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include <stdio.h>
#include <string.h>


//Encoder Setup
const int encoderPinA = 2;
const int encoderPinB = 3;

const double pulsesPerRevolution = 600.0;             //Encoder Pulses per Revolution
const double gearRatio = 16.0;                        //Gear Ratio between wheel gear and encoder gear

double rpm = 0;

Encoder myEncoder(encoderPinA, encoderPinB);

long prevPosition = 0;
unsigned long prevTime = 0;

//GPS Setup
static const int RXPin = 4;
static const int TXPin = 5;
static const uint32_t GPSBaud = 9600;
float lat, lon;

//The TinyGPS++ object
TinyGPSPlus gps;

//The serial connection to the GPS device
SoftwareSerial gpsSerial(RXPin, TXPin);


void setup() 
{
  Serial.begin(9600);
  Wire.begin(4);                                      //Entering I2C Bus as ID 4
  Wire.onRequest(requestEvent);                       //Send data when requested through I2C
  gpsSerial.begin(GPSBaud);                           //TinyGPSPlus Serial Begin
}

void loop() 
{
  //Rotary Encoder Code
  long currentPosition = myEncoder.read();            //Reading Encoder Position
  unsigned long currentTime = millis();               //Reading Current Time

  //Calculating Position Change
  long positionChange = currentPosition - prevPosition; 

  //Calculating Time Change 
  unsigned long timeChange = currentTime - prevTime;  

  if (timeChange > 0) 
  {
    //Calculating Wheel Speed
    double wheelSpeed = static_cast<double>(positionChange) / timeChange;   

    //Convert to RPM with the gear ratio
    rpm = (wheelSpeed * 60.0 * 1000.0) / (pulsesPerRevolution * gearRatio); 

    Serial.print("Wheel RPM: ");
    Serial.println(rpm);

    prevPosition = currentPosition;
    prevTime = currentTime;
  }


  //GPS Code
  // This sketch displays information every time a new sentence is correctly encoded.
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
    while(true);
  }

  delay(100);  // Adjust the delay based on your requirements
}


//GPS Void Function to Display Info
void displayInfo()
{
  //TinyGPSPlus Sending GPS Coordinates in Decimal
  if (gps.location.isValid())
  {
    //Latitude
    Serial.print("Latitude: ");
    lat = gps.location.lat();
    Serial.println(lat, 6);
    //Longitude
    Serial.print("Longitude: ");
    lon = gps.location.lng();
    Serial.println(lon, 6);
    
    //Not using at the moment:
    //Altitude
    //Serial.print("Altitude: ");
    //Serial.println(gps.altitude.meters());
    //Speed in miles per hour (double)
    //Serial.print("Speed in miles/h = ");
    //Serial.println(gps.speed.mph());
  }
  else
  {
    Serial.println("Location: Not Available");
  }

  Serial.println();
  Serial.println();
  delay(100);
}

//I2C to Communicate with the Pi 
void requestEvent()
{
  long lat_scaled = abs(lat) * 1000000;               //Scaling Latitude to be able to send through I2C multiplying by 1e6 to get an int
  long lon_scaled = abs(lon) * 1000000;               //Scaling Longitude to be able to send through I2C multiplying by 1e6 to get an int
  Wire.write((uint8_t*)&lat_scaled, sizeof(lat));     //Sending Scaled Latitude to Raspbery Pi through I2C
  Wire.write((uint8_t*)&lon_scaled, sizeof(lon));     //Sending Scaled Longitude to Raspbery Pi through I2C
  int rpm_scaled = rpm * 100;                         //Scaling RPM Speed to 1e2
  Wire.write((uint8_t*)&rpm_scaled, 1);               //Sending Scaled RPM Speed to Raspbery Pi through I2C

  //Checking Latitude and Longitude for negative values to send ctrl bit
  if (lat > 0 && lon > 0)                             //Case 0 if both Lat and Long are positive
  {
    int ctrl = 0;                                     
    Wire.write((uint8_t*)&ctrl, 1);                   //Send to Raspberry Pi through I2C
  }
  else if (lat > 0 && lon < 0)                        //Case 1 if Lat is positive and Long is negative
  {
    int ctrl = 1;                                     
    Wire.write((uint8_t*)&ctrl, 1);                   //Send to Raspberry Pi through I2C
  }
  else if (lat < 0 && lon > 0)                        //Case 2 if Lat is negative and Long is positive
  {
    int ctrl = 2;                                     
    Wire.write((uint8_t*)&ctrl, 1);                   //Send to Raspberry Pi through I2C
  }
  else if (lat < 0 && lon < 0)                        //Case3 3 if Lat is negative and Long is negative
  {
    int ctrl = 3;                                     
    Wire.write((uint8_t*)&ctrl, 1);                   //Send to Raspberry Pi through I2C
  }

  Serial.println("Received Message");
}