#include "MotorDriver.h"

MotorDriver::MotorDriver(uint8_t pinA, uint8_t pinB)
  : _pinA(pinA), _pinB(pinB) {}

void MotorDriver::begin() {
  pinMode(_pinA, OUTPUT);
  pinMode(_pinB, OUTPUT);
}

void MotorDriver::setSpeed(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) {
    analogWrite(_pinA, speed);
    analogWrite(_pinB, 0);
  } else {
    analogWrite(_pinA, 0);
    analogWrite(_pinB, -speed);
  }
}
