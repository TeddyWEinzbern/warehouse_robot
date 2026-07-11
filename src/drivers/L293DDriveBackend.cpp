#include "drivers/L293DDriveBackend.h"

namespace robot {
namespace {
constexpr uint8_t LatchPin = 12;
constexpr uint8_t ClockPin = 4;
constexpr uint8_t EnablePin = 7;
constexpr uint8_t DataPin = 8;
// Preserve the currently tuned 70 max PWM and +5 rear start threshold.
constexpr L293DDriveBackend::Motor FrontLeft = {3, 1, 4, 30, 70, false};
constexpr L293DDriveBackend::Motor FrontRight = {11, 2, 3, 30, 70, false};
constexpr L293DDriveBackend::Motor RearLeft = {6, 5, 7, 35, 70, false};
constexpr L293DDriveBackend::Motor RearRight = {5, 0, 6, 35, 70, false};
}

L293DDriveBackend::L293DDriveBackend() : latch_(0), feedback_({{0, 0, 0, 0}, 0, 0, false, false}) {}

void L293DDriveBackend::begin() {
    pinMode(LatchPin, OUTPUT);
    pinMode(ClockPin, OUTPUT);
    pinMode(EnablePin, OUTPUT);
    pinMode(DataPin, OUTPUT);
    pinMode(3, OUTPUT); pinMode(5, OUTPUT); pinMode(6, OUTPUT); pinMode(11, OUTPUT);
    digitalWrite(EnablePin, LOW);
    writeLatch();
    stop();
}

void L293DDriveBackend::setBit(uint8_t bit, bool enabled) {
    if (enabled) latch_ |= static_cast<uint8_t>(1U << bit);
    else latch_ &= static_cast<uint8_t>(~(1U << bit));
}

void L293DDriveBackend::writeLatch() {
    digitalWrite(LatchPin, LOW);
    shiftOut(DataPin, ClockPin, MSBFIRST, latch_);
    digitalWrite(LatchPin, HIGH);
}

void L293DDriveBackend::setMotor(const Motor &motor, int16_t speed) {
    long magnitude = speed < 0 ? -static_cast<long>(speed) : speed;
    if (magnitude > 1000) magnitude = 1000;
    if (magnitude == 0) {
        analogWrite(motor.pwmPin, 0);
        setBit(motor.aBit, false); setBit(motor.bBit, false);
        writeLatch();
        return;
    }
    const uint8_t pwm = static_cast<uint8_t>(
        motor.minPwm + magnitude * (motor.maxPwm - motor.minPwm) / 1000L
    );
    bool forward = speed > 0;
    if (motor.reversed) forward = !forward;
    setBit(motor.aBit, forward); setBit(motor.bBit, !forward);
    writeLatch();
    analogWrite(motor.pwmPin, pwm);
}

void L293DDriveBackend::setWheelTargets(const WheelTargets &targets) {
    setMotor(FrontLeft, targets.frontLeft);
    setMotor(FrontRight, targets.frontRight);
    setMotor(RearLeft, targets.rearLeft);
    setMotor(RearRight, targets.rearRight);
}

void L293DDriveBackend::stop() { setWheelTargets({0, 0, 0, 0}); }
void L293DDriveBackend::poll(uint32_t) {}
DriveCapabilities L293DDriveBackend::capabilities() const { return {true, false, false, false}; }
const DriveFeedback &L293DDriveBackend::feedback() const { return feedback_; }

} // namespace robot

