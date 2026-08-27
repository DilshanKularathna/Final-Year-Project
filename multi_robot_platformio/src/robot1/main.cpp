#include <Arduino.h>
#include <MotorDriver.h>   // from lib/MotorDriver — shared across robots
#include <RobotComms.h>    // from lib/RobotComms — shared across robots

MotorDriver leftMotor(25, 26);
RobotComms comms("robot1");

void setup() {
  Serial.begin(115200);
  leftMotor.begin();
}

void loop() {
  leftMotor.setSpeed(150);

  String msg = comms.buildStatusMessage(1.23, 4.56);
  Serial.println(msg);

  delay(1000);
}
