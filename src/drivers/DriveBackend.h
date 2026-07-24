#pragma once

#include "domain/RobotTypes.h"
#include "domain/RuntimeConfig.h"

namespace robot {

class DriveBackend {
  public:
    virtual void begin(const RuntimeConfig &runtime) = 0;
    virtual void pollReceive(uint32_t nowMs, const RuntimeConfig &runtime) = 0;
    virtual void service(uint32_t nowMs, const RuntimeConfig &runtime) = 0;
    virtual void setWheelTargets(const WheelTargets &targets) = 0;
    virtual void onMotorDeadline(uint32_t nowMs, bool armed, const RuntimeConfig &runtime) = 0;
    virtual void onEncoderDeadline(uint32_t nowMs, const RuntimeConfig &runtime) = 0;
    virtual void onEncoderTotalDeadline(uint32_t nowMs) = 0;
    virtual void stop(uint32_t nowMs) = 0;
    virtual const DriveFeedback &feedback() const = 0;
    virtual DriveHealth health(uint32_t nowMs) const = 0;
    virtual void clearFaults() = 0;
    virtual uint8_t outstandingQuery() const = 0;

  protected:
    // Backends have static lifetime and are never deleted through this
    // interface. Keeping the destructor non-virtual avoids pulling the AVR
    // heap allocator in only for deleting-destructor vtable entries.
    ~DriveBackend() {}
};

} // namespace robot
