#pragma once

#include <stdint.h>

namespace robot {

enum Button : uint16_t {
    PresetLeft = 1U << 0,
    PresetFront = 1U << 1,
    PresetRight = 1U << 2,
    PresetStow = 1U << 3,
    StartAssist = 1U << 4,
    CancelAssist = 1U << 5,
    EmergencyStop = 1U << 6,
    ClearEmergencyStop = 1U << 7,
};

enum class IntentSource : uint8_t { Operator, Assist, Safety };
enum class AssistStage : uint8_t { Idle, Align, Range, Complete, Cancelled };
enum class SensorDirection : uint8_t { Front = 0, Left = 1, Right = 2 };

struct OperatorControlFrame {
    int16_t forward;
    int16_t turn;
    int16_t strafe;
    int16_t armYaw;
    int16_t armReach;
    int16_t armHeight;
    int8_t gripper;
    uint16_t buttons;
    uint16_t pressed;
    uint8_t sequence;
    uint32_t receivedAtMs;
    bool valid;
    bool directWheels;
    int16_t wheelFrontLeft;
    int16_t wheelFrontRight;
    int16_t wheelRearLeft;
    int16_t wheelRearRight;
};

struct DriveIntent {
    int16_t forward;
    int16_t turn;
    int16_t strafe;
    uint16_t maxMagnitude;
    IntentSource source;
};

struct WheelTargets {
    int16_t frontLeft;
    int16_t frontRight;
    int16_t rearLeft;
    int16_t rearRight;
};

struct ArmTarget {
    float yawDegrees;
    float reachMm;
    float heightMm;
    uint8_t gripperDegrees;
};

struct DistancePair {
    uint16_t firstMm;
    uint16_t secondMm;
    uint32_t updatedAtMs;
    uint8_t count;
    bool valid;
};

struct SensorSnapshot {
    DistancePair directions[3];
};

struct DriveCapabilities {
    bool openLoop;
    bool closedLoopSpeed;
    bool encoderFeedback;
    bool batteryFeedback;
};

struct DriveFeedback {
    int32_t encoder[4];
    uint16_t batteryMv;
    uint32_t updatedAtMs;
    bool encoderValid;
    bool batteryValid;
};

struct AssistOutput {
    DriveIntent drive;
    float requestedReachMm;
    AssistStage stage;
    bool driveActive;
    bool reachActive;
};

struct RobotStatus {
    AssistStage assistStage;
    bool cargoMayBeHeld;
    bool linkAlive;
    bool emergencyStopped;
    uint16_t faults;
};

} // namespace robot
