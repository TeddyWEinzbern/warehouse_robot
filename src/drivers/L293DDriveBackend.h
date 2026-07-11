#pragma once

#include <Arduino.h>

#include "drivers/DriveBackend.h"

namespace robot {

class L293DDriveBackend : public DriveBackend {
  public:
    struct Motor {
        uint8_t pwmPin;
        uint8_t aBit;
        uint8_t bBit;
        uint8_t minPwm;
        uint8_t maxPwm;
        bool reversed;
    };

    L293DDriveBackend();
    void begin();
    void setWheelTargets(const WheelTargets &targets);
    void stop();
    void poll(uint32_t nowMs);
    DriveCapabilities capabilities() const;
    const DriveFeedback &feedback() const;

  private:
    uint8_t latch_;
    DriveFeedback feedback_;
    void setMotor(const Motor &motor, int16_t speed);
    void setBit(uint8_t bit, bool enabled);
    void writeLatch();
};

} // namespace robot
