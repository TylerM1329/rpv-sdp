#include <Wire.h>
// using ATMEGA328P old bootloader for prototype

// Pin definitions for motor and steering control
const char R_EN = 2;  // Right enable pin
const char RPWM = 3;  // Right PWM control pin
const char L_EN = 4;  // Left enable pin
const char LPWM = 5;  // Left PWM control pin
const char steering_pot = A0;  // Analog input for the steering potentiometer

// Buffer for receiving data
char recvBuf[4];

// Timing variables
unsigned long currTimeDelay = 0;
const short int delayFreq = 50;  // Loop frequency in milliseconds

// Steering position variables
short int targetPos = 300;   // Default target position
short int tempTarget = 300;  // Temporary target position
short int currentPos = 0;    // Current potentiometer reading
short int deadband = 10;     // Range around the target position for no movement

void setup() {
  // Set motor control pins as outputs
  pinMode(R_EN, OUTPUT);
  pinMode(RPWM, OUTPUT);
  pinMode(L_EN, OUTPUT);
  pinMode(LPWM, OUTPUT);
  Serial.begin(115200);

  // Initialize I2C with slave address 8
  Wire.begin(8);
  Wire.onReceive(receiveEvent);

  // Enable motor drivers
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);
}

void loop() {
  // Update position at defined intervals
  if (millis() >= currTimeDelay + delayFreq) {
    currTimeDelay += delayFreq;

    // Read the current position from the steering potentiometer
    currentPos = analogRead(steering_pot);

    // Check if within the deadband of the target position
    if ((currentPos <= targetPos + deadband) && (currentPos >= targetPos - deadband)) {
      digitalWrite(RPWM, 0);  // Stop the motor
      digitalWrite(LPWM, 0);
    }
    // Current position is greater than the target, move left
    if (currentPos > targetPos + deadband) {
      digitalWrite(RPWM, 1);
      digitalWrite(LPWM, 0);
    }
    // Current position is less than the target, move right
    if (currentPos < targetPos - deadband) {
      digitalWrite(RPWM, 0);
      digitalWrite(LPWM, 1);
    }
  }
}

// I2C receive event handler
void receiveEvent() {
  // Map incoming byte (0-100) to target position range (103-392)
  tempTarget = map(Wire.read(), 0, 100, 103, 392);
  if (tempTarget > 392) {
    targetPos = 300;  // Reset to default if the value is out of range
  } else {
    targetPos = tempTarget;
  }
  
  Serial.println(targetPos);  // Output the target position for debugging
}
