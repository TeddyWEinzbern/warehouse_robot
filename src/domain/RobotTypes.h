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
};

enum ControlFlag : uint8_t { EStopAsserted = 1U << 0 };
enum RequestFlag : uint8_t {
    RequestArm = 1U << 0,
    RequestDisarm = 1U << 1,
    RequestClearEStop = 1U << 2,
    RequestClearFault = 1U << 3,
    RequestHello = 1U << 4,
};

enum class RobotState : uint8_t { Boot, Disarmed, Armed, EStop, Fault };
enum class RobotProfile : uint8_t {
    // Slots 0-2 and 4-5 belong to retired profiles (safe idle, L293D,
    // qualification, open-loop calibration, arm-only calibration); they are
    // kept so the wire values of the surviving profiles never change.
    SafeIdle,
    L293DDevelopment,
    UartClosedLoopQualification,
    UartClosedLoopRobot,
    UartOpenLoopCalibration,
    ArmCalibration,
    UartOpenLoopRobot,
    Calibration,
};
enum class DriveControlMode : uint8_t {
    None,
    L293DOpenLoopPwm, // retired backend; slot kept for host protocol stability
    UartOpenLoopPwm,
    UartClosedLoopSpeed,
};
enum class PwmUnit : uint8_t { Unavailable, Raw8Bit, PercentX100 };
enum class EncoderSampleSemantics : uint8_t { ProvisionalFixed20Ms, ElapsedBetweenSamples };
enum class ResponseProfile : uint8_t { Low, Normal, Aggressive };
enum class IntentSource : uint8_t { Operator, Assist, Safety };
enum class AssistStage : uint8_t { Idle, Align, Range, Complete, Cancelled };
enum class SensorDirection : uint8_t { Front = 0, Left = 1, Right = 2 };

enum FaultCode : uint16_t {
    FaultNone = 0,
    FaultSchedulerOverrun = 1U << 0,
    FaultDriveInitialization = 1U << 1,
    FaultEncoderStale = 1U << 2,
    FaultEncoderMalformed = 1U << 3,
    FaultEncoderImplausible = 1U << 4,
    FaultEncoderSign = 1U << 5,
    FaultDriveStall = 1U << 6,
    FaultDriveMismatch = 1U << 7,
    FaultArmTarget = 1U << 8,
};

enum WarningCode : uint16_t {
    WarningNone = 0,
    WarningDriveUnqualified = 1U << 0,
    WarningEncoderSignCandidate = 1U << 1,
    WarningEncoderScaleCandidate = 1U << 2,
    WarningSensorStale = 1U << 3,
    WarningArmTargetLimited = 1U << 4,
};

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
    uint8_t controlFlags;
    uint8_t sequence;
    uint32_t receivedAtMs;
    bool valid;
};

struct ControlRequests { uint8_t flags; };

struct DriveIntent {
    int16_t forward;
    int16_t turn;
    int16_t strafe;
    uint16_t maxMagnitudePermille;
    IntentSource source;
};

struct ChassisVelocity {
    int16_t longitudinalMmS;
    int16_t lateralMmS;
    int16_t yawMradS;
};

struct ChassisAccelerationLimits {
    uint16_t forwardAccelMmS2;
    uint16_t forwardDecelMmS2;
    uint16_t reverseAccelMmS2;
    uint16_t reverseDecelMmS2;
    uint16_t lateralAccelMmS2;
    uint16_t lateralDecelMmS2;
    uint16_t rotationalAccelMradS2;
    uint16_t rotationalDecelMradS2;
    uint16_t zeroCrossingHoldMs;
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

struct DistanceReading {
    uint16_t millimetres;
    uint32_t updatedAtMs;
    bool valid;
};

struct DistancePair {
    DistanceReading first;
    DistanceReading second;
    uint8_t count;
};

struct SensorSnapshot { DistancePair directions[3]; };

struct DriveCapabilities {
    DriveControlMode mode;
    PwmUnit pwmUnit;
    bool encoderFeedback;
    bool batteryFeedback;
    bool internalPwmFeedback;
};

struct DriveFeedback {
    int16_t controllerTargetMmS[4];
    int16_t rawIncrement[4];
    int32_t total[4];
    int16_t measuredMmS[4];
    int16_t errorMmS[4];
    int16_t openLoopPwm[4];
    uint16_t batteryMv;
    uint32_t incrementUpdatedAtMs;
    uint32_t totalUpdatedAtMs;
    uint32_t batteryUpdatedAtMs;
    uint16_t sampleIntervalMs;
    uint8_t encoderValidMask;
    uint8_t totalValidMask;
    uint8_t errorValidMask;
    bool batteryValid;
    EncoderSampleSemantics semantics;
};

struct DriveHealth {
    uint16_t faults;
    uint16_t warnings;
    bool initialized;
    bool feedbackReady;
    bool feedbackHealthy;
};

struct AssistOutput {
    DriveIntent drive;
    float requestedReachMm;
    AssistStage stage;
    bool driveActive;
    bool reachActive;
};

struct SchedulerTaskStats {
    uint16_t missed;
    uint16_t consecutiveMisses;
    uint32_t maxLatenessUs;
};

struct RobotStatus {
    RobotState state;
    AssistStage assistStage;
    uint16_t faults;
    uint16_t warnings;
    uint16_t commandAgeMs;
    bool cargoMayBeHeld;
    bool linkAlive;
    bool emergencyStopped;
    bool driveCalibrated;
};

} // namespace robot
