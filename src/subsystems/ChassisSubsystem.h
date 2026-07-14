#pragma once

#include "domain/RuntimeConfig.h"
#include "drivers/DriveBackend.h"

namespace robot {

class ChassisSubsystem {
  public:
    explicit ChassisSubsystem(DriveBackend &backend);
    void begin(const RuntimeConfig &runtime);
    void setDesired(const DriveIntent &intent, const RuntimeConfig &runtime);
    void trajectoryTick(uint32_t nowUs, uint32_t elapsedUs, const RuntimeConfig &runtime);
    void motorTick(uint32_t nowMs, bool armed, const RuntimeConfig &runtime);
    void forceZero(uint32_t nowMs);
    const ChassisVelocity &requestedVelocity() const;
    const ChassisVelocity &rampedVelocity() const;
    const WheelTargets &wheelTargets() const;
    uint8_t zeroCrossingMask(uint32_t nowUs) const;
    DriveBackend &backend();

  private:
    struct AxisState {
        int16_t value;
        uint32_t zeroHoldUntilUs;
    };

    DriveBackend &backend_;
    ChassisVelocity requested_;
    ChassisVelocity ramped_;
    WheelTargets wheels_;
    AxisState longitudinal_;
    AxisState lateral_;
    AxisState yaw_;
    static int16_t scaleAxis(int16_t input, int16_t positiveMaximum, int16_t negativeMaximum);
    static int16_t rampAxis(
        AxisState &state,
        int16_t target,
        uint16_t positiveAcceleration,
        uint16_t negativeAcceleration,
        uint16_t positiveDeceleration,
        uint16_t negativeDeceleration,
        uint16_t zeroThreshold,
        uint16_t zeroHoldMs,
        uint32_t nowUs,
        uint32_t elapsedUs
    );
};

} // namespace robot
