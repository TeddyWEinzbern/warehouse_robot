#pragma once

#include <stdint.h>

#include "domain/RobotTypes.h"

namespace robot {

enum class ParameterGroup : uint8_t {
    Servo = 1,
    // 2 was OpenLoopMotor for the retired L293D backend; keep the slot free.
    ChassisSpeed = 3,
    ChassisAcceleration = 4,
    Encoder = 5,
    Sensor = 6,
    Assist = 7,
    ResponseProfile = 8,
    UartOpenLoop = 9,
    ArmGeometry = 10,
};

struct ServoCalibration {
    uint8_t lowerDegrees;
    uint8_t upperDegrees;
    int8_t centerOffsetDegrees;
    int8_t direction;
};

struct MotorCalibration {
    uint8_t minimumPwm;
    uint8_t maximumPwm;
    int8_t direction;
};

struct ResponseProfileDefinition {
    uint16_t speedPermille;
    uint16_t accelerationPermille;
    uint16_t decelerationPermille;
};

struct EncoderCalibration {
    uint16_t wheelDiameterMm;
    uint16_t countsPerRevolution;
    uint16_t wheelTrackMm;
    uint16_t wheelbaseMm;
    int8_t channelMap[4];
    int8_t signs[4];
    int8_t commandMap[4];
    int8_t commandSigns[4];
    EncoderSampleSemantics semantics;
};

struct ChassisLimits {
    int16_t maximumForwardMmS;
    int16_t maximumReverseMmS;
    int16_t maximumLateralMmS;
    int16_t maximumYawMradS;
    uint16_t maximumWheelMmS;
    ChassisAccelerationLimits acceleration;
    uint16_t translationZeroThresholdMmS;
    uint16_t rotationZeroThresholdMradS;
    ResponseProfile activeProfile;
};

struct ArmGeometry {
    uint16_t firstLinkMm;
    uint16_t secondLinkMm;
    uint16_t shoulderBaseHeightMm;
    uint16_t gripperLengthOffsetMm;
    uint16_t minimumReachMm;
    uint16_t maximumReachMm;
    uint16_t minimumHeightMm;
    uint16_t maximumHeightMm;
    uint16_t cargoClearanceHeightMm;
    uint16_t presetReachMm;
    uint16_t presetHeightMm;
    uint16_t stowReachMm;
    uint16_t stowHeightMm;
};

struct RuntimeConfig {
    ServoCalibration servos[4];
    MotorCalibration uartOpenLoop[4];
    ResponseProfileDefinition responseProfiles[3];
    EncoderCalibration encoder;
    ChassisLimits chassis;
    ArmGeometry arm;
    int16_t sensorOffsetMm[6];
    uint16_t normalDriveLimitPermille;
    uint16_t cargoDriveLimitPermille;
    uint16_t assistDriveLimitPermille;
    uint16_t revision;

    static RuntimeConfig defaults();
    bool validate() const;
    bool applyParameter(
        ParameterGroup group,
        uint8_t index,
        const uint8_t *data,
        uint8_t length,
        bool aggressiveAllowed
    );
};

} // namespace robot
