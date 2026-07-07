#pragma once

#include <Arduino.h>

enum class RobotMode : uint8_t {
    Manual,
    Automatic,
};

enum class RobotCommandType : uint8_t {
    None,
    Drive,
    WheelSpeeds,
    SingleMotor,
};

struct DriveCommand {
    int forward;
    int turn;
    int strafe;
};

struct WheelSpeedCommand {
    int frontLeft;
    int frontRight;
    int rearLeft;
    int rearRight;
};

struct SingleMotorCommand {
    uint8_t motorIndex;
    int speed;
};

struct RobotCommand {
    RobotCommandType type;
    DriveCommand drive;
    WheelSpeedCommand wheels;
    SingleMotorCommand singleMotor;
};

struct CommandStats {
    unsigned long bytesReceived;
    unsigned long validCommands;
    unsigned long ignoredCommands;
};
