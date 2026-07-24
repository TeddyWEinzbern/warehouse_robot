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
    void stop(uint32_t) {}
    const DriveFeedback &feedback() const { return feedback_; }
    DriveHealth health(uint32_t) const { return {0, 0, true, true, true}; }
    void clearFaults() {}
    uint8_t outstandingQuery() const { return 0; }
  private:
    DriveFeedback feedback_;
};
} // namespace robot
