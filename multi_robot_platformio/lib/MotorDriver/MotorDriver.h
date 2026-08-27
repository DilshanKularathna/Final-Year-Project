#pragma once
#include <Arduino.h>

// Example shared module. Any robot's src/ can #include <MotorDriver.h>
// and it will be linked in automatically — no lib_deps entry needed
// for local libraries in lib/.

class MotorDriver {
  public:
    MotorDriver(uint8_t pinA, uint8_t pinB);
    void begin();
    void setSpeed(int speed); // -255..255, negative = reverse

  private:
    uint8_t _pinA;
    uint8_t _pinB;
};
