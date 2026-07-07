#include "MotorController.h"

#include <math.h>

namespace {

float absFloat(float value) {
    return value < 0.0f ? -value : value;
}

float maxFloat(float a, float b) {
    return a > b ? a : b;
}

uint8_t minByte(uint8_t a, uint8_t b) {
    return a < b ? a : b;
}

} // namespace

MotorController::MotorController()
    : latchState_(0), lastWheelSpeeds_({0, 0, 0, 0}), stopped_(true) {
}

void MotorController::begin() {
    pinMode(Config::MOTOR_LATCH_PIN, OUTPUT);
    pinMode(Config::MOTOR_CLOCK_PIN, OUTPUT);
    pinMode(Config::MOTOR_ENABLE_PIN, OUTPUT);
    pinMode(Config::MOTOR_DATA_PIN, OUTPUT);

    pinMode(Config::M1_PWM_PIN, OUTPUT);
    pinMode(Config::M2_PWM_PIN, OUTPUT);
    pinMode(Config::M3_PWM_PIN, OUTPUT);
    pinMode(Config::M4_PWM_PIN, OUTPUT);

    digitalWrite(Config::MOTOR_ENABLE_PIN, LOW);
    latchState_ = 0;
    updateLatch();
    stop();
}

void MotorController::stop() {
    setWheelSpeeds(0, 0, 0, 0);
    stopped_ = true;
}

void MotorController::drive(int forward, int turn, int strafe) {
    if (Config::REVERSE_FORWARD_COMMAND) {
        forward = -forward;
    }
    if (Config::REVERSE_TURN_COMMAND) {
        turn = -turn;
    }
    if (Config::REVERSE_STRAFE_COMMAND) {
        strafe = -strafe;
    }

    if (Config::ACTIVE_DRIVE_MIXING == Config::DriveMixing::Mecanum) {
        driveMecanum(forward, turn, strafe);
    } else {
        driveTank(forward, turn);
    }
}

void MotorController::setWheelSpeeds(
    int frontLeft, int frontRight, int rearLeft, int rearRight
) {
    setMotorSpeed(Config::FRONT_LEFT_MOTOR, frontLeft);
    setMotorSpeed(Config::FRONT_RIGHT_MOTOR, frontRight);
    setMotorSpeed(Config::REAR_LEFT_MOTOR, rearLeft);
    setMotorSpeed(Config::REAR_RIGHT_MOTOR, rearRight);

    lastWheelSpeeds_.frontLeft = frontLeft;
    lastWheelSpeeds_.frontRight = frontRight;
    lastWheelSpeeds_.rearLeft = rearLeft;
    lastWheelSpeeds_.rearRight = rearRight;
    stopped_ =
        frontLeft == 0 && frontRight == 0 && rearLeft == 0 && rearRight == 0;
}

void MotorController::setSingleMotor(uint8_t motorIndex, int speed) {
    setWheelSpeeds(
        motorIndex == 1 ? speed : 0,
        motorIndex == 2 ? speed : 0,
        motorIndex == 3 ? speed : 0,
        motorIndex == 4 ? speed : 0
    );
}

const WheelSpeedCommand &MotorController::lastWheelSpeeds() const {
    return lastWheelSpeeds_;
}

bool MotorController::isStopped() const {
    return stopped_;
}

void MotorController::updateLatch() {
    digitalWrite(Config::MOTOR_LATCH_PIN, LOW);
    shiftOut(
        Config::MOTOR_DATA_PIN, Config::MOTOR_CLOCK_PIN, MSBFIRST, latchState_
    );
    digitalWrite(Config::MOTOR_LATCH_PIN, HIGH);
}

void MotorController::setLatchBit(uint8_t bit, bool enabled) {
    if (enabled) {
        latchState_ |= static_cast<uint8_t>(1U << bit);
    } else {
        latchState_ &= static_cast<uint8_t>(~(1U << bit));
    }
}

uint8_t MotorController::speedToPwm(
    const Config::MotorConfig &motor, float speed
) const {
    float magnitude = absFloat(speed) * motor.speedScale;
    if (magnitude < 0.5f) {
        return 0;
    }
    if (magnitude > 1000.0f) {
        magnitude = 1000.0f;
    }

    const uint8_t maxPwm = motor.maxPwm;
    const uint8_t minPwm = minByte(motor.minPwm, maxPwm);
    float pwm = minPwm + (magnitude * (maxPwm - minPwm) / 1000.0f);
    if (pwm > maxPwm) {
        pwm = maxPwm;
    }
    return static_cast<uint8_t>(pwm + 0.5f);
}

void MotorController::setMotorSpeed(
    const Config::MotorConfig &motor, float speed
) {
    const uint8_t pwm = speedToPwm(motor, speed);
    if (pwm == 0) {
        analogWrite(motor.pwmPin, 0);
        setLatchBit(motor.aLatchBit, false);
        setLatchBit(motor.bLatchBit, false);
        updateLatch();
        return;
    }

    bool forward = speed > 0.0f;
    if (motor.reversePolarity) {
        forward = !forward;
    }

    setLatchBit(motor.aLatchBit, forward);
    setLatchBit(motor.bLatchBit, !forward);
    updateLatch();
    analogWrite(motor.pwmPin, pwm);
}

void MotorController::normalize(
    float &frontLeft, float &frontRight, float &rearLeft, float &rearRight
) const {
    const float maxMagnitude = maxFloat(
        maxFloat(absFloat(frontLeft), absFloat(frontRight)),
        maxFloat(absFloat(rearLeft), absFloat(rearRight))
    );

    if (maxMagnitude <= 1000.0f) {
        return;
    }

    frontLeft = frontLeft * 1000.0f / maxMagnitude;
    frontRight = frontRight * 1000.0f / maxMagnitude;
    rearLeft = rearLeft * 1000.0f / maxMagnitude;
    rearRight = rearRight * 1000.0f / maxMagnitude;
}

void MotorController::driveTank(int forward, int turn) {
    float left = forward + turn;
    float right = forward - turn;
    float frontLeft = left;
    float frontRight = right;
    float rearLeft = left;
    float rearRight = right;

    normalize(frontLeft, frontRight, rearLeft, rearRight);
    setWheelSpeeds(frontLeft, frontRight, rearLeft, rearRight);
}

void MotorController::driveMecanum(int forward, int turn, int strafe) {
    float frontLeft = forward + strafe + turn;
    float frontRight = forward - strafe - turn;
    float rearLeft = forward - strafe + turn;
    float rearRight = forward + strafe - turn;

    normalize(frontLeft, frontRight, rearLeft, rearRight);
    setWheelSpeeds(frontLeft, frontRight, rearLeft, rearRight);
}
