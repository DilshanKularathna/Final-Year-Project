#include <Arduino.h>

// TODO: robot6-specific logic goes here.
// #include shared modules from lib/ as needed, e.g.:
// #include <MotorDriver.h>
// #include <RobotComms.h>

void setup() {
  Serial.begin(115200);
  Serial.println("robot6 booted");
}

void loop() {
  delay(1000);
}
