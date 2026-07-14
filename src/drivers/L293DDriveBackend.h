#pragma once

#include <Arduino.h>

#include "drivers/DriveBackend.h"

namespace robot {

class L293DDriveBackend : public DriveBackend {
  public:
    struct Motor { uint8_t pwmPin; uint8_t aBit; uint8_t bBit; };
    L293DDriveBackend();
    void begin(const RuntimeConfig &runtime);
    void pollReceive(uint32_t, const RuntimeConfig &) {}
    void service(uint32_t nowMs, const RuntimeConfig &runtime);
    void setWheelTargets(const WheelTargets &targets);
    void onMotorDeadline(uint32_t nowMs, bool armed, const RuntimeConfig &runtime);
    void onEncoderDeadline(uint32_t, const RuntimeConfig &) {}
    void onEncoderTotalDeadline(uint32_t) {}
    void onBatteryDeadline(uint32_t) {}
    void stop(uint32_t nowMs);
    DriveCapabilities capabilities() const;
    const DriveFeedback &feedback() const;
    DriveHealth health(uint32_t nowMs) const;
    void clearFaults() {}
    uint16_t queryTimeouts() const { return 0; }
    uint16_t rxOverflows() const { return 0; }
    uint16_t motorCommandAgeMs(uint32_t nowMs) const;
    uint8_t outstandingQuery() const { return 0; }
    uint16_t outstandingQueryAgeMs(uint32_t) const { return 0; }

  private:
    struct MotorState {
        int8_t appliedSign;
        int16_t pendingSpeed;
        uint32_t reverseReadyMs;
        bool reversing;
    };
    uint8_t latch_;
    DriveFeedback feedback_;
    WheelTargets targets_;
    MotorState states_[4];
    uint32_t lastMotorCommandAtMs_;
    void serviceMotor(uint8_t index, int16_t speed, uint32_t nowMs, const RuntimeConfig &runtime);
    void applyMotor(uint8_t index, int16_t speed, const RuntimeConfig &runtime);
    void setBit(uint8_t bit, bool enabled);
    void writeLatch();
};

} // namespace robot
