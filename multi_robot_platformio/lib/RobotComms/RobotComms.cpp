#include "RobotComms.h"

RobotComms::RobotComms(const char* robotId) : _robotId(robotId) {}

String RobotComms::buildStatusMessage(float value1, float value2) {
  JsonDocument doc;
  doc["robot"] = _robotId;
  doc["value1"] = value1;
  doc["value2"] = value2;
  doc["uptime_ms"] = millis();

  String out;
  serializeJson(doc, out);
  return out;
}
