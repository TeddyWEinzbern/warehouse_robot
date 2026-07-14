#include "subsystems/ChassisSubsystem.h"

#include "app/BuildConfig.h"
#include "domain/MecanumKinematics.h"

namespace robot {
namespace {
int32_t absolute32(int32_t value) { return value < 0 ? -value : value; }
int16_t approach(int16_t current, int16_t target, int32_t step) {
    if (step <= 0) return current;
    if (target > current && static_cast<int32_t>(target) - current > step)
        return static_cast<int16_t>(current + step);
    if (target < current && static_cast<int32_t>(current) - target > step)
        return static_cast<int16_t>(current - step);
    return target;
}
bool oppositeSigns(int16_t a, int16_t b) { return (a < 0 && b > 0) || (a > 0 && b < 0); }
uint16_t profiled(uint16_t value, uint16_t multiplierPermille, uint16_t hardMaximum) {
    const uint32_t scaled = static_cast<uint32_t>(value) * multiplierPermille / 1000UL;
    return static_cast<uint16_t>(scaled > hardMaximum ? hardMaximum : scaled);
}
int16_t speedProfiled(int16_t value, uint16_t multiplierPermille) {
    return static_cast<int16_t>(
        static_cast<int32_t>(value) * multiplierPermille / 1000L
    );
}
}

ChassisSubsystem::ChassisSubsystem(DriveBackend &backend)
    : backend_(backend), requested_({0, 0, 0}), ramped_({0, 0, 0}), wheels_({0, 0, 0, 0}),
      longitudinal_({0, 0}), lateral_({0, 0}), yaw_({0, 0}) {}

void ChassisSubsystem::begin(const RuntimeConfig &runtime) { backend_.begin(runtime); }

int16_t ChassisSubsystem::scaleAxis(int16_t input, int16_t positiveMaximum, int16_t negativeMaximum) {
    if (input >= 0) return static_cast<int16_t>(static_cast<int32_t>(input) * positiveMaximum / 1000L);
    return static_cast<int16_t>(static_cast<int32_t>(input) * negativeMaximum / 1000L);
}

void ChassisSubsystem::setDesired(const DriveIntent &intent, const RuntimeConfig &runtime) {
    const ResponseProfile profile = runtime.chassis.activeProfile;
    const ResponseProfileDefinition &definition =
        runtime.responseProfiles[static_cast<uint8_t>(profile)];
    const int16_t forwardMaximum = speedProfiled(
        runtime.chassis.maximumForwardMmS, definition.speedPermille
    );
    const int16_t reverseMaximum = speedProfiled(
        runtime.chassis.maximumReverseMmS, definition.speedPermille
    );
    const int16_t lateralMaximum = speedProfiled(
        runtime.chassis.maximumLateralMmS, definition.speedPermille
    );
    const int16_t yawMaximum = speedProfiled(
        runtime.chassis.maximumYawMradS, definition.speedPermille
    );
    requested_.longitudinalMmS = scaleAxis(intent.forward, forwardMaximum, reverseMaximum);
    requested_.lateralMmS = scaleAxis(intent.strafe, lateralMaximum, lateralMaximum);
    requested_.yawMradS = scaleAxis(intent.turn, yawMaximum, yawMaximum);
    requested_.longitudinalMmS = static_cast<int16_t>(
        static_cast<int32_t>(requested_.longitudinalMmS) * intent.maxMagnitudePermille / 1000L
    );
    requested_.lateralMmS = static_cast<int16_t>(
        static_cast<int32_t>(requested_.lateralMmS) * intent.maxMagnitudePermille / 1000L
    );
    requested_.yawMradS = static_cast<int16_t>(
        static_cast<int32_t>(requested_.yawMradS) * intent.maxMagnitudePermille / 1000L
    );
}

int16_t ChassisSubsystem::rampAxis(
    AxisState &state, int16_t target,
    uint16_t positiveAcceleration, uint16_t negativeAcceleration,
    uint16_t positiveDeceleration, uint16_t negativeDeceleration,
    uint16_t zeroThreshold, uint16_t zeroHoldMs,
    uint32_t nowUs, uint32_t elapsedUs
) {
    if (oppositeSigns(state.value, target)) {
        const uint16_t limit = state.value > 0 ? positiveDeceleration : negativeDeceleration;
        state.value = approach(state.value, 0, static_cast<uint32_t>(limit) * elapsedUs / 1000000UL);
        if (absolute32(state.value) <= zeroThreshold) {
            state.value = 0;
            state.zeroHoldUntilUs = nowUs + static_cast<uint32_t>(zeroHoldMs) * 1000UL;
        }
        return state.value;
    }
    if (state.value == 0 && target != 0 && static_cast<int32_t>(nowUs - state.zeroHoldUntilUs) < 0)
        return 0;
    const bool accelerating = absolute32(target) > absolute32(state.value);
    uint16_t limit;
    if (accelerating) limit = target >= 0 ? positiveAcceleration : negativeAcceleration;
    else limit = state.value >= 0 ? positiveDeceleration : negativeDeceleration;
    state.value = approach(state.value, target, static_cast<uint32_t>(limit) * elapsedUs / 1000000UL);
    if (target == 0 && absolute32(state.value) <= zeroThreshold) state.value = 0;
    return state.value;
}

void ChassisSubsystem::trajectoryTick(uint32_t nowUs, uint32_t elapsedUs, const RuntimeConfig &runtime) {
    const ChassisAccelerationLimits &base = runtime.chassis.acceleration;
    const ResponseProfile profile = runtime.chassis.activeProfile;
    const ResponseProfileDefinition &definition =
        runtime.responseProfiles[static_cast<uint8_t>(profile)];
    ramped_.longitudinalMmS = rampAxis(
        longitudinal_, requested_.longitudinalMmS,
        profiled(base.forwardAccelMmS2, definition.accelerationPermille,
                 config::HardMaxTranslationAccelerationMmS2),
        profiled(base.reverseAccelMmS2, definition.accelerationPermille,
                 config::HardMaxTranslationAccelerationMmS2),
        profiled(base.forwardDecelMmS2, definition.decelerationPermille,
                 config::HardMaxTranslationAccelerationMmS2),
        profiled(base.reverseDecelMmS2, definition.decelerationPermille,
                 config::HardMaxTranslationAccelerationMmS2),
        runtime.chassis.translationZeroThresholdMmS, base.zeroCrossingHoldMs, nowUs, elapsedUs
    );
    ramped_.lateralMmS = rampAxis(
        lateral_, requested_.lateralMmS,
        profiled(base.lateralAccelMmS2, definition.accelerationPermille,
                 config::HardMaxTranslationAccelerationMmS2),
        profiled(base.lateralAccelMmS2, definition.accelerationPermille,
                 config::HardMaxTranslationAccelerationMmS2),
        profiled(base.lateralDecelMmS2, definition.decelerationPermille,
                 config::HardMaxTranslationAccelerationMmS2),
        profiled(base.lateralDecelMmS2, definition.decelerationPermille,
                 config::HardMaxTranslationAccelerationMmS2),
        runtime.chassis.translationZeroThresholdMmS, base.zeroCrossingHoldMs, nowUs, elapsedUs
    );
    ramped_.yawMradS = rampAxis(
        yaw_, requested_.yawMradS,
        profiled(base.rotationalAccelMradS2, definition.accelerationPermille,
                 config::HardMaxRotationalAccelerationMradS2),
        profiled(base.rotationalAccelMradS2, definition.accelerationPermille,
                 config::HardMaxRotationalAccelerationMradS2),
        profiled(base.rotationalDecelMradS2, definition.decelerationPermille,
                 config::HardMaxRotationalAccelerationMradS2),
        profiled(base.rotationalDecelMradS2, definition.decelerationPermille,
                 config::HardMaxRotationalAccelerationMradS2),
        runtime.chassis.rotationZeroThresholdMradS, base.zeroCrossingHoldMs, nowUs, elapsedUs
    );
    uint16_t wheelLimit = runtime.chassis.maximumWheelMmS;
#if ROBOT_DRIVE_QUALIFICATION
    if (wheelLimit > config::QualificationWheelLimitMmS) wheelLimit = config::QualificationWheelLimitMmS;
#endif
    wheels_ = MecanumKinematics::mix(
        ramped_, runtime.encoder.wheelTrackMm, runtime.encoder.wheelbaseMm, wheelLimit
    );
    backend_.setWheelTargets(wheels_);
}

void ChassisSubsystem::motorTick(uint32_t nowMs, bool armed, const RuntimeConfig &runtime) {
    backend_.onMotorDeadline(nowMs, armed, runtime);
}

void ChassisSubsystem::forceZero(uint32_t nowMs) {
    requested_ = {0, 0, 0};
    ramped_ = {0, 0, 0};
    wheels_ = {0, 0, 0, 0};
    longitudinal_ = {0, 0}; lateral_ = {0, 0}; yaw_ = {0, 0};
    backend_.setWheelTargets(wheels_);
    backend_.stop(nowMs);
}

const ChassisVelocity &ChassisSubsystem::requestedVelocity() const { return requested_; }
const ChassisVelocity &ChassisSubsystem::rampedVelocity() const { return ramped_; }
const WheelTargets &ChassisSubsystem::wheelTargets() const { return wheels_; }
uint8_t ChassisSubsystem::zeroCrossingMask(uint32_t nowUs) const {
    uint8_t mask = 0;
    if (longitudinal_.value == 0 &&
        static_cast<int32_t>(nowUs - longitudinal_.zeroHoldUntilUs) < 0) mask |= 1U;
    if (lateral_.value == 0 &&
        static_cast<int32_t>(nowUs - lateral_.zeroHoldUntilUs) < 0) mask |= 2U;
    if (yaw_.value == 0 &&
        static_cast<int32_t>(nowUs - yaw_.zeroHoldUntilUs) < 0) mask |= 4U;
    return mask;
}
DriveBackend &ChassisSubsystem::backend() { return backend_; }

} // namespace robot
