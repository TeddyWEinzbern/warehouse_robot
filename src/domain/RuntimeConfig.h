#pragma once

#include <stdint.h>

#include "domain/RobotTypes.h"

namespace robot {

struct ServoCalibration {
    uint8_t lowerDegrees;
    uint8_t upperDegrees;
    int8_t centerOffsetDegrees;
    int8_t direction;
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
    EncoderCalibration encoder;
    ChassisLimits chassis;
    ArmGeometry arm;

    static RuntimeConfig defaults();
    bool setCalibrationServoReference(
        uint8_t joint,
        uint8_t lowerDegrees,
        uint8_t upperDegrees,
        int8_t centerOffsetDegrees,
        int8_t direction
    );
};

} // namespace robot
