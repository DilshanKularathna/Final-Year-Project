#include <Arduino.h>

// TODO: robot4-specific logic goes here.
// #include shared modules from lib/ as needed, e.g.:
// #include <MotorDriver.h>
// #include <RobotComms.h>

void setup() {
  Serial.begin(115200);
  Serial.println("robot4 booted");
}

void loop() {
  delay(1000);
}
