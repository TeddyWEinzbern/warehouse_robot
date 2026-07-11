#pragma once

#include "domain/RobotTypes.h"

namespace robot {

class DriveBackend {
  public:
    virtual ~DriveBackend() {}
    virtual void begin() = 0;
    virtual void setWheelTargets(const WheelTargets &targets) = 0;
    virtual void stop() = 0;
    virtual void poll(uint32_t nowMs) = 0;
    virtual DriveCapabilities capabilities() const = 0;
    virtual const DriveFeedback &feedback() const = 0;
};

} // namespace robot

