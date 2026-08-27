#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>

// Example shared comms module — e.g. a common JSON message format
// that every robot uses to talk to a base station over MQTT/WiFi.

class RobotComms {
  public:
    RobotComms(const char* robotId);
    String buildStatusMessage(float value1, float value2);

  private:
    const char* _robotId;
};
