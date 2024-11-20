#include <Wire.h>

// Define the pins for the ultrasonic sensors
const int backTrigPin = 9;   // Trigger pin for the back sensor
const int backEchoPin = 8;   // Echo pin for the back sensor
const int frontTrigPin = 5;  // Trigger pin for the front sensor
const int frontEchoPin = 4;  // Echo pin for the front sensor
// Uncomment these lines if left and right sensors are used in the future
// const int leftTrigPin = 7;
// const int leftEchoPin = 6;
// const int rightTrigPin = 3;
// const int rightEchoPin = 2;

// Variables to store distance measurements
int frontDistance, backDistance;
// int leftDistance, rightDistance; // Uncomment for left and right sensors
char dataOut[4]; // Data buffer for I2C communication

void setup() {
  Wire.begin(3);                  // Set up I2C with slave address 3
  Wire.onRequest(requestEvent);   // Register function to handle I2C requests
  Serial.begin(115200);           // Initialize serial communication for debugging
  
  // Configure pins for front and back sensors
  pinMode(frontTrigPin, OUTPUT);
  pinMode(frontEchoPin, INPUT);
  pinMode(backTrigPin, OUTPUT);
  pinMode(backEchoPin, INPUT);

  // Uncomment for additional sensors
  // pinMode(leftTrigPin, OUTPUT);
  // pinMode(leftEchoPin, INPUT);
  // pinMode(rightTrigPin, OUTPUT);
  // pinMode(rightEchoPin, INPUT);
}  

void requestEvent() {
  // Prepare distance data for I2C
  if (frontDistance > 255)
    dataOut[0] = 255; // Cap at 255 to fit within a single byte
  else
    dataOut[0] = frontDistance;

  if (backDistance > 255)
    dataOut[1] = 255; // Cap at 255 to fit within a single byte
  else
    dataOut[1] = backDistance;

  // Uncomment for additional sensors
  // dataOut[2] = leftDistance;
  // dataOut[3] = rightDistance;

  // Write distance data to the I2C bus
  Wire.write(dataOut, 2); // Sending only front and back distances
  Serial.println("I2C data requested. Sending:");
  Serial.print("Front: ");
  Serial.print(dataOut[0], DEC);
  Serial.print(" Back: ");
  Serial.print(dataOut[1], DEC);
  Serial.println();
}

void loop() {
  // Measure distances
  frontDistance = measureDistance(frontTrigPin, frontEchoPin);
  backDistance = measureDistance(backTrigPin, backEchoPin);

  // Uncomment for additional sensors
  // leftDistance = measureDistance(leftTrigPin, leftEchoPin);
  // rightDistance = measureDistance(rightTrigPin, rightEchoPin);

  // Debugging: Print the measured distances
  Serial.print("Front: ");
  Serial.print(frontDistance);
  Serial.print(" Back: ");
  Serial.println(backDistance);

  // Delay before the next reading
  delay(50);
}

// Function to measure the distance using an ultrasonic sensor
int measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH); // Measure the duration of the echo pulse
  int distance = duration * 0.0343 / 2;  // Convert duration to distance in cm
  return distance;
}

// (Optional) Function to print sensor data in a formatted table
void printSensorData(String sensorName, int distance) {
  String status;
  if (distance > 790) { // Handle invalid or very close readings
    distance = 0;
    status = "Very Close";       
  } else {
    status = "Normal";
  }
  Serial.print("| " + sensorName + "      |  ");
  Serial.print(distance);
  Serial.print(" cm            |  ");
  Serial.println(status + "  |");
}

/*
  Developer Notes:
  0. **Deprecation Notice**:
     - This hardware configuration and associated code are **deprecated** and no longer in active use.
     - Any future modifications should consider transitioning to a newer hardware setup and updated codebase.
     
  1. **I2C Data Limitation**:
     - Each distance is sent as a single byte (0–255). Distances above 255 are capped at 255.
     - If greater precision or a wider range is needed, consider using multiple bytes per distance.

  2. **Uncommented Sensors**:
     - Left and right sensors are not currently implemented but can be added by uncommenting the relevant sections.

  3. **Sensor Limitations**:
     - The ultrasonic sensors may produce inaccurate readings for objects too close (<2 cm) or too far (>400 cm).
     - The `pulseIn()` function may return zero if no echo is detected, causing a zero distance.

  4. **Debugging Output**:
     - Debug messages are printed via Serial. Remove or redirect these in the final deployment.

  5. **Timing**:
     - Adjust the `delay(50)` in the loop to balance responsiveness and stability of the system.

  6. **Future Improvements**:
     - Consider implementing averaging or filtering for distance values to reduce noise and enhance stability.
*/
