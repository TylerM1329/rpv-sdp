#include <Wire.h>

#define I2C_SLAVE_ADDRESS 6 // Set the Arduino's I2C address

// Define pins for controlling the lights
const int headlightsPin = 2;
const int leftTurnSignalPin = 3;
const int rightTurnSignalPin = 4;
const int brakeLightsPin = 5;
const int CVlightsPin = 6;

int receivedAction = 0; // Variable to store the received action
bool blinkLeftSignal = false;
bool blinkRightSignal = false;
bool activateHazards = false;

void setup() {
  Wire.begin(I2C_SLAVE_ADDRESS);
  Wire.onReceive(receiveEvent);

  // Initialize pins as outputs
  pinMode(headlightsPin, OUTPUT);
  pinMode(leftTurnSignalPin, OUTPUT);
  pinMode(rightTurnSignalPin, OUTPUT);
  pinMode(brakeLightsPin, OUTPUT);
  pinMode(CVlightsPin, OUTPUT);

  // Turn off all lights initially
  turnOffLights();
}

void loop() {
if (activateHazards) {
    // Toggle both left and right turn signals for hazard lights
    digitalWrite(leftTurnSignalPin, !digitalRead(leftTurnSignalPin));
    digitalWrite(rightTurnSignalPin, !digitalRead(rightTurnSignalPin));
    delay(400);  // Control the blink rate of hazard lights
  } else {
    // Handle left turn signal
    if (blinkLeftSignal) {
      digitalWrite(leftTurnSignalPin, !digitalRead(leftTurnSignalPin));
      delay(400);
    } else{
      digitalWrite(leftTurnSignalPin, LOW);
    }
  
    // Handle right turn signal
    if (blinkRightSignal) {
      digitalWrite(rightTurnSignalPin, !digitalRead(rightTurnSignalPin));
      delay(400);
    } else {
      digitalWrite(rightTurnSignalPin, LOW);
    }
  }
}

void receiveEvent(int byteCount) {
  if (byteCount == 1) {  // Expecting a single byte for the action
    receivedAction = Wire.read();  // Read the received action

    // Process the received action
    switch (receivedAction) {
      case 0:  // Turn on headlights
        digitalWrite(headlightsPin, HIGH);
        break;
      case 1:  // Turn off headlights
        digitalWrite(headlightsPin, LOW);
        break;
      case 2:  // Toggle left turn signal
        blinkLeftSignal = !blinkLeftSignal;
        break;
      case 4:  // Activate right turn signal
        blinkRightSignal = !blinkRightSignal;
        break;
      case 5:  // Toggle hazard lights
        activateHazards = !activateHazards;
        break;
      case 6:  // Apply brake lights
        digitalWrite(brakeLightsPin, HIGH);
        break;
      case 7:  // Release brake lights
        digitalWrite(brakeLightsPin, LOW);
        break;
      case 8:  // Turn on CV lights
        digitalWrite(CVlightsPin, HIGH);
        break;
      case 9:  // Turn off CV lights
        digitalWrite(CVlightsPin, LOW);
        break;
      default:
        // Handle unknown action or add custom cases as needed
        break;
    }
  }
}

void turnOffLights() {
  digitalWrite(headlightsPin, LOW);
  digitalWrite(leftTurnSignalPin, LOW);
  digitalWrite(rightTurnSignalPin, LOW);
  digitalWrite(brakeLightsPin, LOW);
  digitalWrite(CVlightsPin, LOW);
}
