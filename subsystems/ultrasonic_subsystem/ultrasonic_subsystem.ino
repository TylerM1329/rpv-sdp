#include <Wire.h>

// Define the pins for the ultrasonic sensors
const int backTrigPin = 9;
const int backEchoPin = 8;
//const int leftTrigPin = 7;
//const int leftEchoPin = 6;
//const int rightTrigPin = 3;
//const int rightEchoPin = 2;
const int frontTrigPin = 5;    
const int frontEchoPin = 4;
int frontDistance, leftDistance, rightDistance, backDistance;
char dataOut[4];

void setup() {
  Wire.begin(3);
  Wire.onRequest(requestEvent);
  Serial.begin(115200);
  
  pinMode(frontTrigPin, OUTPUT);
  pinMode(frontEchoPin, INPUT);
  // pinMode(leftTrigPin, OUTPUT);
  // pinMode(leftEchoPin, INPUT);
  // pinMode(rightTrigPin, OUTPUT);
  // pinMode(rightEchoPin, INPUT);
  pinMode(backTrigPin, OUTPUT);
  pinMode(backEchoPin, INPUT);
}  
void requestEvent()
{
  // copy distance data into dataOut array
  if (frontDistance > 255)
    dataOut[0] = 255;
  else
    dataOut[0] = frontDistance;

  if (backDistance > 255)
    dataOut[1] = 255;
  else
    dataOut[1] = backDistance;
  // dataOut[2] = leftDistance;
  // dataOut[3] = rightDistance;
  // write dataOut array to I2C bus
  Wire.write(dataOut, 2);
  Serial.println("I2C data requested. Sending:");
  Serial.print("Front: ");
  Serial.print(dataOut[0], DEC);
  Serial.print(" Back: ");
  Serial.print(dataOut[1], DEC);
  Serial.println();

}

void loop() {
  // Initialize variables to store distances
  //int frontDistance, leftDistance, rightDistance, backDistance;

  // Measure distances
  frontDistance = measureDistance(frontTrigPin, frontEchoPin);
  // leftDistance = measureDistance(leftTrigPin, leftEchoPin);
  // rightDistance = measureDistance(rightTrigPin, rightEchoPin);
  backDistance = measureDistance(backTrigPin, backEchoPin);

  // Display data as a table
  /*Serial.println("=========================================");
  Serial.println("| Sensor   |  Distance (cm)  |  Status   |");
  Serial.println("=========================================");
  printSensorData("Front", frontDistance);
  printSensorData("Left", leftDistance);
  printSensorData("Right", rightDistance);
  printSensorData("Back", backDistance);
  Serial.println("=========================================");*/
  
  Serial.print("Front: ");
  Serial.print(frontDistance);
  Serial.print(" Back: ");
  Serial.print(backDistance);
  Serial.println();

  // Delay before the next reading
  delay(50);
}

int measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  int distance = duration * 0.0343 / 2;

  return distance;
}
void printSensorData(String sensorName, int distance) {
  String status;
  if (distance > 790) {
    distance = 0; // Set distance to a large value
    status = "Very Close";       
  } else {
    status = "Normal";
  }
  Serial.print("| " + sensorName + "      |  ");
  Serial.print(distance);
  Serial.print(" cm            |  ");
  Serial.println(status + "  |");
}