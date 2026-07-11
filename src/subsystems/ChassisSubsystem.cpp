#include "subsystems/ChassisSubsystem.h"

#include "domain/MecanumKinematics.h"

namespace robot {

ChassisSubsystem::ChassisSubsystem(DriveBackend &backend)
    : backend_(backend), last_({0, 0, 0, 0}), lastUpdateMs_(0), stopped_(true) {}

void ChassisSubsystem::begin() { backend_.begin(); backend_.stop(); }

int16_t ChassisSubsystem::clampAxis(long value, uint16_t limit) {
    if (value > limit) return static_cast<int16_t>(limit);
    if (value < -static_cast<long>(limit)) return -static_cast<int16_t>(limit);
    return static_cast<int16_t>(value);
}

int16_t ChassisSubsystem::approach(int16_t current, int16_t target, int16_t step) {
    if (target > current + step) return current + step;
    if (target < current - step) return current - step;
    return target;
}

void ChassisSubsystem::update(const DriveIntent &intent, uint32_t nowMs) {
    backend_.poll(nowMs);
    if (nowMs - lastUpdateMs_ < 20UL) return;
    WheelTargets target = MecanumKinematics::mix(
        clampAxis(intent.forward, intent.maxMagnitude),
        clampAxis(intent.turn, intent.maxMagnitude),
        clampAxis(intent.strafe, intent.maxMagnitude)
    );
    uint32_t elapsed = nowMs - lastUpdateMs_;
    if (elapsed > 100) elapsed = 100;
    const int16_t step = static_cast<int16_t>(20 + elapsed * 4);
    last_.frontLeft = approach(last_.frontLeft, target.frontLeft, step);
    last_.frontRight = approach(last_.frontRight, target.frontRight, step);
    last_.rearLeft = approach(last_.rearLeft, target.rearLeft, step);
    last_.rearRight = approach(last_.rearRight, target.rearRight, step);
    backend_.setWheelTargets(last_);
    stopped_ = last_.frontLeft == 0 && last_.frontRight == 0 &&
               last_.rearLeft == 0 && last_.rearRight == 0;
    lastUpdateMs_ = nowMs;
}

void ChassisSubsystem::updateDirect(
    const WheelTargets &targets, uint16_t limit, uint32_t nowMs
) {
    backend_.poll(nowMs);
    if (nowMs - lastUpdateMs_ < 20UL) return;
    last_.frontLeft = clampAxis(targets.frontLeft, limit);
    last_.frontRight = clampAxis(targets.frontRight, limit);
    last_.rearLeft = clampAxis(targets.rearLeft, limit);
    last_.rearRight = clampAxis(targets.rearRight, limit);
    backend_.setWheelTargets(last_);
    stopped_ = last_.frontLeft == 0 && last_.frontRight == 0 &&
               last_.rearLeft == 0 && last_.rearRight == 0;
    lastUpdateMs_ = nowMs;
}

void ChassisSubsystem::stop() {
    if (!stopped_) backend_.stop();
    last_ = {0, 0, 0, 0};
    stopped_ = true;
}
DriveBackend &ChassisSubsystem::backend() { return backend_; }

} // namespace robot
