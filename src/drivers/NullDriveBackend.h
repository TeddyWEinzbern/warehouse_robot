#pragma once

#include "drivers/DriveBackend.h"

namespace robot {
class NullDriveBackend : public DriveBackend {
  public:
    NullDriveBackend() : feedback_({{0, 0, 0, 0}, 0, 0, false, false}) {}
    void begin() {}
    void setWheelTargets(const WheelTargets &) {}
    void stop() {}
    void poll(uint32_t) {}
    DriveCapabilities capabilities() const { return {false, false, false, false}; }
    const DriveFeedback &feedback() const { return feedback_; }
  private:
    DriveFeedback feedback_;
};
} // namespace robot
