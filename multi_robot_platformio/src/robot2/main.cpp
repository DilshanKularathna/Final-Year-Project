#include <Arduino.h>
#include <RobotComms.h>    // reuse shared comms module

RobotComms comms("robot2");

void setup() {
  Serial.begin(115200);
  // e.g. TFT/display setup specific to this robot goes here
}

void loop() {
  String msg = comms.buildStatusMessage(9.87, 6.54);
  Serial.println(msg);
  delay(1000);
}
