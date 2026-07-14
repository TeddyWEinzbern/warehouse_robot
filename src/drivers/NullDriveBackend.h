#pragma once

#include "drivers/DriveBackend.h"

namespace robot {
class NullDriveBackend : public DriveBackend {
  public:
    NullDriveBackend() : feedback_({}) {}
    void begin(const RuntimeConfig &) {}
    void pollReceive(uint32_t, const RuntimeConfig &) {}
    void service(uint32_t, const RuntimeConfig &) {}
    void setWheelTargets(const WheelTargets &) {}
    void onMotorDeadline(uint32_t, bool, const RuntimeConfig &) {}
    void onEncoderDeadline(uint32_t, const RuntimeConfig &) {}
    void onEncoderTotalDeadline(uint32_t) {}
    void onBatteryDeadline(uint32_t) {}
    void stop(uint32_t) {}
    DriveCapabilities capabilities() const {
        return {DriveControlMode::None, PwmUnit::Unavailable, false, false, false};
    }
    const DriveFeedback &feedback() const { return feedback_; }
    DriveHealth health(uint32_t) const { return {0, 0, true, true, true}; }
    void clearFaults() {}
    uint16_t queryTimeouts() const { return 0; }
    uint16_t rxOverflows() const { return 0; }
    uint16_t motorCommandAgeMs(uint32_t) const { return 0; }
    uint8_t outstandingQuery() const { return 0; }
    uint16_t outstandingQueryAgeMs(uint32_t) const { return 0; }
  private:
    DriveFeedback feedback_;
};
} // namespace robot
