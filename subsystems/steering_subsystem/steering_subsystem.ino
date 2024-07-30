#include <Wire.h>
// using ATMEGA328P old bootloader for prototype

const char R_EN = 2;
const char RPWM = 3;
const char L_EN = 4;
const char LPWM = 5;
const char steering_pot = A0;

char recvBuf[4];

unsigned long currTimeDelay = 0;
const short int delayFreq = 50;
short int targetPos = 300;
short int tempTarget = 300;
short int currentPos = 0;
short int deadband = 10;

void setup() {
  pinMode(R_EN, OUTPUT);
  pinMode(RPWM, OUTPUT);
  pinMode(L_EN, OUTPUT);
  pinMode(LPWM, OUTPUT);
  Serial.begin(115200);

  Wire.begin(8);
  Wire.onReceive(receiveEvent);

  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);
}

void loop() {
  /*if (Serial.available()) {
    for (int i = 0; i < 4; i++) {
      recvBuf[i] = Serial.read();
      delay(1);
    }
    targetPos = atoi(recvBuf);
    Serial.println(targetPos);
  }*/
  if (millis() >= currTimeDelay + delayFreq) {
    currTimeDelay += delayFreq;
    currentPos = analogRead(steering_pot);

    if ((currentPos <= targetPos + deadband) && (currentPos >= targetPos - deadband)) {
      digitalWrite(RPWM, 0);
      digitalWrite(LPWM, 0);
    }

    if (currentPos > targetPos + deadband) {
      digitalWrite(RPWM, 1);
      digitalWrite(LPWM, 0);
    }

    if (currentPos < targetPos - deadband) {
      digitalWrite(RPWM, 0);
      digitalWrite(LPWM, 1);
    }
  }
}

void receiveEvent() {
  //targetPos = Wire.read();
  tempTarget = map(Wire.read(), 0, 100, 103, 392);
  if (tempTarget > 392) {
    targetPos = 300;
  } else
    targetPos = tempTarget;
  Serial.println(targetPos);
}
