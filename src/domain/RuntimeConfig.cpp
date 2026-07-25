#include "domain/RuntimeConfig.h"

#include "app/BuildConfig.h"

namespace robot {

RuntimeConfig RuntimeConfig::defaults() {
    RuntimeConfig result = {};
    for (uint8_t index = 0; index < 4; ++index) {
        result.servos[index] = {
            config::ServoLowerDegrees[index],
            config::ServoUpperDegrees[index],
            static_cast<int8_t>(config::ServoCenterDegrees[index] - 90),
            config::ServoDirectionSign[index]
        };
        result.encoder.channelMap[index] = config::EncoderChannelMap[index];
        result.encoder.signs[index] = config::EncoderDirectionSign[index];
        result.encoder.commandMap[index] = config::MotorCommandMap[index];
        result.encoder.commandSigns[index] = config::MotorCommandSign[index];
    }
    result.encoder.wheelDiameterMm = config::WheelDiameterMm;
    result.encoder.countsPerRevolution = config::EncoderCountsPerRevolution;
    result.encoder.wheelTrackMm = config::WheelTrackMm;
    result.encoder.wheelbaseMm = config::WheelbaseMm;
    result.encoder.semantics = EncoderSampleSemantics::ProvisionalFixed20Ms;
    result.chassis.maximumForwardMmS = 1000;
    result.chassis.maximumReverseMmS = 1000;
    result.chassis.maximumLateralMmS = 1000;
    result.chassis.maximumYawMradS = 3000;
    result.chassis.maximumWheelMmS = config::HardWheelLimitMmS;
    result.chassis.acceleration = {500, 800, 400, 800, 400, 700, 1200, 2000, 40};
    result.chassis.translationZeroThresholdMmS = 10;
    result.chassis.rotationZeroThresholdMradS = 20;
    result.arm = {
        static_cast<uint16_t>(config::FirstLinkMm),
        static_cast<uint16_t>(config::SecondLinkMm),
        static_cast<uint16_t>(config::ShoulderBaseHeightMm),
        static_cast<uint16_t>(config::GripperLengthOffsetMm),
        static_cast<uint16_t>(config::MinReachMm),
        static_cast<uint16_t>(config::MaxReachMm),
        static_cast<uint16_t>(config::MinHeightMm),
        static_cast<uint16_t>(config::MaxHeightMm),
        static_cast<uint16_t>(config::CargoClearanceHeightMm),
        static_cast<uint16_t>(config::PresetReachMm),
        static_cast<uint16_t>(config::PresetHeightMm),
        static_cast<uint16_t>(config::StowReachMm),
        static_cast<uint16_t>(config::StowHeightMm)
    };
    return result;
}

bool RuntimeConfig::setCalibrationServoReference(
    uint8_t joint, uint8_t lowerDegrees, uint8_t upperDegrees,
    int8_t centerOffsetDegrees, int8_t direction
) {
    if (joint >= 4 || lowerDegrees >= upperDegrees || upperDegrees > 180 ||
        centerOffsetDegrees < -90 || centerOffsetDegrees > 90 ||
        (direction != 1 && direction != -1))
        return false;
    const int16_t centerDegrees =
        static_cast<int16_t>(centerOffsetDegrees) + 90;
    if (centerDegrees < lowerDegrees || centerDegrees > upperDegrees)
        return false;
    servos[joint] = {
        lowerDegrees, upperDegrees, centerOffsetDegrees, direction
    };
    return true;
}

} // namespace robot
