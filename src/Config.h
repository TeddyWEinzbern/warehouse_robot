#pragma once

#include <Arduino.h>
#include <stddef.h>

namespace Config {

enum class CommandInterface : uint8_t {
    Usb,
    Bluetooth,
    Both,
};

enum class DriveMixing : uint8_t {
    Tank,
    Mecanum,
};

struct MotorConfig {
    uint8_t pwmPin;
    uint8_t aLatchBit;
    uint8_t bLatchBit;
    bool reversePolarity;
    uint8_t minPwm;
    uint8_t maxPwm;
    float speedScale;
};

constexpr CommandInterface ACTIVE_COMMAND_INTERFACE = CommandInterface::Both;
constexpr DriveMixing ACTIVE_DRIVE_MIXING = DriveMixing::Mecanum;

constexpr unsigned long USB_BAUD = 115200;
constexpr unsigned long BLUETOOTH_BAUD = 9600;

constexpr uint8_t BLUETOOTH_RX_PIN = A5;
constexpr uint8_t BLUETOOTH_TX_PIN = A4;

constexpr bool REVERSE_FORWARD_COMMAND = false;
constexpr bool REVERSE_TURN_COMMAND = false;
constexpr bool REVERSE_STRAFE_COMMAND = false;

constexpr uint8_t MOTOR_LATCH_PIN = 12;
constexpr uint8_t MOTOR_CLOCK_PIN = 4;
constexpr uint8_t MOTOR_ENABLE_PIN = 7;
constexpr uint8_t MOTOR_DATA_PIN = 8;

constexpr uint8_t M1_PWM_PIN = 11;
constexpr uint8_t M2_PWM_PIN = 3;
constexpr uint8_t M3_PWM_PIN = 6;
constexpr uint8_t M4_PWM_PIN = 5;

constexpr uint8_t M1_A_LATCH_BIT = 2;
constexpr uint8_t M1_B_LATCH_BIT = 3;
constexpr uint8_t M2_A_LATCH_BIT = 1;
constexpr uint8_t M2_B_LATCH_BIT = 4;
constexpr uint8_t M3_A_LATCH_BIT = 5;
constexpr uint8_t M3_B_LATCH_BIT = 7;
constexpr uint8_t M4_A_LATCH_BIT = 0;
constexpr uint8_t M4_B_LATCH_BIT = 6;

constexpr uint8_t DEFAULT_MAX_MOTOR_PWM = 100;
constexpr uint8_t DEFAULT_MIN_MOTOR_PWM = 30;

constexpr MotorConfig FRONT_RIGHT_MOTOR = {
    M1_PWM_PIN,
    M1_A_LATCH_BIT,
    M1_B_LATCH_BIT,
    false,
    DEFAULT_MIN_MOTOR_PWM,
    DEFAULT_MAX_MOTOR_PWM,
    1.00F,
};

constexpr MotorConfig FRONT_LEFT_MOTOR = {
    M2_PWM_PIN,
    M2_A_LATCH_BIT,
    M2_B_LATCH_BIT,
    false,
    DEFAULT_MIN_MOTOR_PWM,
    DEFAULT_MAX_MOTOR_PWM,
    1.00F,
};

constexpr MotorConfig REAR_LEFT_MOTOR = {
    M3_PWM_PIN,
    M3_A_LATCH_BIT,
    M3_B_LATCH_BIT,
    false,
    DEFAULT_MIN_MOTOR_PWM,
    DEFAULT_MAX_MOTOR_PWM,
    1.00F,
};

constexpr MotorConfig REAR_RIGHT_MOTOR = {
    M4_PWM_PIN,
    M4_A_LATCH_BIT,
    M4_B_LATCH_BIT,
    false,
    DEFAULT_MIN_MOTOR_PWM,
    DEFAULT_MAX_MOTOR_PWM,
    1.00F,
};

constexpr unsigned long COMMAND_TIMEOUT_MS = 300;
constexpr bool DEBUG_SERIAL = false;
constexpr unsigned long DEBUG_INTERVAL_MS = 1000;
constexpr size_t COMMAND_BUFFER_SIZE = 40;

// Reserved for future arm modules. Kept here so mechanical limits do not get
// scattered when shoulder/elbow/base/gripper servos are added.
constexpr uint8_t SERVO_MIN_ANGLE = 0;
constexpr uint8_t SERVO_MAX_ANGLE = 180;

} // namespace Config
