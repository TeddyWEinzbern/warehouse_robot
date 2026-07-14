#pragma once

#include "domain/RobotTypes.h"
#include "domain/RuntimeConfig.h"

namespace robot {

class DriveBackend {
  public:
    virtual ~DriveBackend() {}
    virtual void begin(const RuntimeConfig &runtime) = 0;
    virtual void pollReceive(uint32_t nowMs, const RuntimeConfig &runtime) = 0;
    virtual void service(uint32_t nowMs, const RuntimeConfig &runtime) = 0;
    virtual void setWheelTargets(const WheelTargets &targets) = 0;
    virtual void onMotorDeadline(uint32_t nowMs, bool armed, const RuntimeConfig &runtime) = 0;
    virtual void onEncoderDeadline(uint32_t nowMs, const RuntimeConfig &runtime) = 0;
    virtual void onEncoderTotalDeadline(uint32_t nowMs) = 0;
    virtual void onBatteryDeadline(uint32_t nowMs) = 0;
    virtual void stop(uint32_t nowMs) = 0;
    virtual DriveCapabilities capabilities() const = 0;
    virtual const DriveFeedback &feedback() const = 0;
    virtual DriveHealth health(uint32_t nowMs) const = 0;
    virtual void clearFaults() = 0;
    virtual uint16_t queryTimeouts() const = 0;
    virtual uint16_t rxOverflows() const = 0;
    virtual uint16_t motorCommandAgeMs(uint32_t nowMs) const = 0;
    virtual uint8_t outstandingQuery() const = 0;
    virtual uint16_t outstandingQueryAgeMs(uint32_t nowMs) const = 0;
};

} // namespace robot
