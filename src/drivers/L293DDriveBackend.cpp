#include "drivers/L293DDriveBackend.h"

namespace robot {
namespace {
constexpr uint8_t LatchPin = 12;
constexpr uint8_t ClockPin = 4;
constexpr uint8_t EnablePin = 7;
constexpr uint8_t DataPin = 8;
constexpr L293DDriveBackend::Motor Motors[4] = {
    {3, 1, 4}, {11, 2, 3}, {6, 5, 7}, {5, 0, 6}
};
int32_t absolute32(int32_t value) { return value < 0 ? -value : value; }
}

L293DDriveBackend::L293DDriveBackend()
    : latch_(0), feedback_({}), targets_({0, 0, 0, 0}), states_{},
      lastMotorCommandAtMs_(0) {}

void L293DDriveBackend::begin(const RuntimeConfig &) {
    pinMode(LatchPin, OUTPUT); pinMode(ClockPin, OUTPUT);
    pinMode(EnablePin, OUTPUT); pinMode(DataPin, OUTPUT);
    pinMode(3, OUTPUT); pinMode(5, OUTPUT); pinMode(6, OUTPUT); pinMode(11, OUTPUT);
    digitalWrite(EnablePin, LOW);
    writeLatch();
    stop(millis());
}
void L293DDriveBackend::setBit(uint8_t bit, bool enabled) {
    if (enabled) latch_ |= static_cast<uint8_t>(1U << bit);
    else latch_ &= static_cast<uint8_t>(~(1U << bit));
}
void L293DDriveBackend::writeLatch() {
    digitalWrite(LatchPin, LOW); shiftOut(DataPin, ClockPin, MSBFIRST, latch_); digitalWrite(LatchPin, HIGH);
}

void L293DDriveBackend::applyMotor(uint8_t index, int16_t speed, const RuntimeConfig &runtime) {
    const Motor &motor = Motors[index];
    const MotorCalibration &calibration = runtime.motors[index];
    int32_t magnitude = absolute32(speed);
    if (magnitude > runtime.chassis.maximumWheelMmS) magnitude = runtime.chassis.maximumWheelMmS;
    if (magnitude == 0) {
        analogWrite(motor.pwmPin, 0);
        setBit(motor.aBit, false); setBit(motor.bBit, false); writeLatch();
        feedback_.openLoopPwm[index] = 0;
        states_[index].appliedSign = 0;
        return;
    }
    const uint8_t pwm = static_cast<uint8_t>(calibration.minimumPwm +
        magnitude * (calibration.maximumPwm - calibration.minimumPwm) /
        runtime.chassis.maximumWheelMmS);
    bool forward = speed > 0;
    if (calibration.direction < 0) forward = !forward;
    setBit(motor.aBit, forward); setBit(motor.bBit, !forward); writeLatch();
    analogWrite(motor.pwmPin, pwm);
    feedback_.openLoopPwm[index] = static_cast<int16_t>(forward ? pwm : -pwm);
    states_[index].appliedSign = speed > 0 ? 1 : -1;
}

void L293DDriveBackend::serviceMotor(
    uint8_t index, int16_t speed, uint32_t nowMs, const RuntimeConfig &runtime
) {
    MotorState &state = states_[index];
    const int8_t requestedSign = speed > 0 ? 1 : speed < 0 ? -1 : 0;
    if (!state.reversing && state.appliedSign != 0 && requestedSign != 0 && requestedSign != state.appliedSign) {
        applyMotor(index, 0, runtime);
        state.reversing = true;
        state.pendingSpeed = speed;
        state.reverseReadyMs = nowMs + 20UL;
        return;
    }
    if (state.reversing) {
        state.pendingSpeed = speed;
        if (static_cast<int32_t>(nowMs - state.reverseReadyMs) < 0) return;
        state.reversing = false;
        speed = state.pendingSpeed;
    }
    applyMotor(index, speed, runtime);
}

void L293DDriveBackend::service(uint32_t nowMs, const RuntimeConfig &runtime) {
    for (uint8_t index = 0; index < 4; ++index)
        if (states_[index].reversing) serviceMotor(index, states_[index].pendingSpeed, nowMs, runtime);
}
void L293DDriveBackend::setWheelTargets(const WheelTargets &targets) { targets_ = targets; }
void L293DDriveBackend::onMotorDeadline(uint32_t nowMs, bool armed, const RuntimeConfig &runtime) {
    const int16_t values[4] = {
        armed ? targets_.frontLeft : 0, armed ? targets_.frontRight : 0,
        armed ? targets_.rearLeft : 0, armed ? targets_.rearRight : 0
    };
    for (uint8_t index = 0; index < 4; ++index) {
        feedback_.controllerTargetMmS[index] = values[index];
        serviceMotor(index, values[index], nowMs, runtime);
    }
    lastMotorCommandAtMs_ = nowMs;
}
void L293DDriveBackend::stop(uint32_t) {
    targets_ = {0, 0, 0, 0};
    for (uint8_t index = 0; index < 4; ++index) {
        analogWrite(Motors[index].pwmPin, 0);
        setBit(Motors[index].aBit, false); setBit(Motors[index].bBit, false);
        states_[index] = {0, 0, 0, false};
        feedback_.openLoopPwm[index] = 0;
        feedback_.controllerTargetMmS[index] = 0;
    }
    writeLatch();
}
DriveCapabilities L293DDriveBackend::capabilities() const {
    return {DriveControlMode::L293DOpenLoopPwm, PwmUnit::Raw8Bit, false, false, false};
}
const DriveFeedback &L293DDriveBackend::feedback() const { return feedback_; }
DriveHealth L293DDriveBackend::health(uint32_t) const { return {0, 0, true, true, true}; }
uint16_t L293DDriveBackend::motorCommandAgeMs(uint32_t nowMs) const {
    const uint32_t age = nowMs - lastMotorCommandAtMs_;
    return static_cast<uint16_t>(age > 65535UL ? 65535UL : age);
}

} // namespace robot
