#include "domain/RuntimeConfig.h"

#include "app/BuildConfig.h"

namespace robot {
namespace {
int16_t readI16(const uint8_t *data) {
    return static_cast<int16_t>(static_cast<uint16_t>(data[0]) |
                                (static_cast<uint16_t>(data[1]) << 8));
}
uint16_t readU16(const uint8_t *data) { return static_cast<uint16_t>(readI16(data)); }
}

RuntimeConfig RuntimeConfig::defaults() {
    RuntimeConfig result = {};
    const uint8_t centers[4] = {
        config::BaseZeroDegrees, config::ShoulderZeroDegrees,
        config::ElbowZeroDegrees, config::GripperOpenDegrees
    };
    for (uint8_t index = 0; index < 4; ++index) {
        result.servos[index] = {0, 180, static_cast<int8_t>(centers[index] - 90), 1};
        result.uartOpenLoop[index] = {0, 100, 1};
        result.encoder.channelMap[index] = static_cast<int8_t>(index);
        result.encoder.commandMap[index] = static_cast<int8_t>(index);
        result.encoder.commandSigns[index] = 1;
    }
    result.encoder.signs[0] = -1;
    result.encoder.signs[1] = 1;
    result.encoder.signs[2] = -1;
    result.encoder.signs[3] = 1;
    result.encoder.wheelDiameterMm = 60;
    result.encoder.countsPerRevolution = 4680;
    result.encoder.wheelTrackMm = 160;
    result.encoder.wheelbaseMm = 170;
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
#if ROBOT_DRIVE_QUALIFICATION
    result.chassis.activeProfile = ResponseProfile::Low;
#else
    result.chassis.activeProfile = ResponseProfile::Normal;
#endif
    result.responseProfiles[0] = {500, 500, 500};
    result.responseProfiles[1] = {1000, 1000, 1000};
    result.responseProfiles[2] = {1000, 1500, 1250};
    result.normalDriveLimitPermille = config::NormalDriveLimitPermille;
    result.cargoDriveLimitPermille = config::CargoDriveLimitPermille;
    result.assistDriveLimitPermille = config::AssistDriveLimitPermille;
    result.revision = 0;
    return result;
}

bool RuntimeConfig::validate() const {
    uint8_t feedbackMapMask = 0;
    uint8_t commandMapMask = 0;
    for (uint8_t index = 0; index < 4; ++index) {
        if (servos[index].lowerDegrees >= servos[index].upperDegrees ||
            servos[index].upperDegrees > 180 ||
            servos[index].centerOffsetDegrees < -90 ||
            servos[index].centerOffsetDegrees > 90 ||
            (servos[index].direction != 1 && servos[index].direction != -1)) return false;
        if (uartOpenLoop[index].minimumPwm > uartOpenLoop[index].maximumPwm ||
            uartOpenLoop[index].maximumPwm > 100 ||
            (uartOpenLoop[index].direction != 1 && uartOpenLoop[index].direction != -1))
            return false;
        if (encoder.channelMap[index] < 0 || encoder.channelMap[index] > 3 ||
            (encoder.signs[index] != 1 && encoder.signs[index] != -1) ||
            encoder.commandMap[index] < 0 || encoder.commandMap[index] > 3 ||
            (encoder.commandSigns[index] != 1 && encoder.commandSigns[index] != -1))
            return false;
        feedbackMapMask |= static_cast<uint8_t>(1U << encoder.channelMap[index]);
        commandMapMask |= static_cast<uint8_t>(1U << encoder.commandMap[index]);
    }
    for (uint8_t index = 0; index < 3; ++index) {
        if (responseProfiles[index].speedPermille > 1000 ||
            responseProfiles[index].accelerationPermille > 1500 ||
            responseProfiles[index].decelerationPermille > 1500) return false;
    }
    for (uint8_t index = 0; index < 6; ++index)
        if (sensorOffsetMm[index] < -500 || sensorOffsetMm[index] > 500) return false;
    if (static_cast<uint8_t>(chassis.activeProfile) >
            static_cast<uint8_t>(ResponseProfile::Aggressive) ||
        static_cast<uint8_t>(encoder.semantics) >
            static_cast<uint8_t>(EncoderSampleSemantics::ElapsedBetweenSamples)) return false;
    if (feedbackMapMask != 0x0F || commandMapMask != 0x0F) return false;
    if (encoder.wheelDiameterMm == 0 ||
        encoder.wheelDiameterMm > config::HardMaxWheelDiameterMm ||
        encoder.countsPerRevolution == 0 ||
        encoder.countsPerRevolution > config::HardMaxCountsPerRevolution ||
        encoder.wheelTrackMm == 0 ||
        encoder.wheelTrackMm > config::HardMaxWheelGeometryMm ||
        encoder.wheelbaseMm == 0 ||
        encoder.wheelbaseMm > config::HardMaxWheelGeometryMm) return false;
    if (chassis.maximumForwardMmS < 0 || chassis.maximumForwardMmS > config::HardMaxTranslationMmS ||
        chassis.maximumReverseMmS < 0 || chassis.maximumReverseMmS > config::HardMaxTranslationMmS ||
        chassis.maximumLateralMmS < 0 || chassis.maximumLateralMmS > config::HardMaxTranslationMmS ||
        chassis.maximumYawMradS < 0 || chassis.maximumYawMradS > config::HardMaxYawMradS ||
        chassis.maximumWheelMmS == 0 || chassis.maximumWheelMmS > config::HardWheelLimitMmS) return false;
    const ChassisAccelerationLimits &a = chassis.acceleration;
    if (a.forwardAccelMmS2 > config::HardMaxTranslationAccelerationMmS2 ||
        a.forwardDecelMmS2 > config::HardMaxTranslationAccelerationMmS2 ||
        a.reverseAccelMmS2 > config::HardMaxTranslationAccelerationMmS2 ||
        a.reverseDecelMmS2 > config::HardMaxTranslationAccelerationMmS2 ||
        a.lateralAccelMmS2 > config::HardMaxTranslationAccelerationMmS2 ||
        a.lateralDecelMmS2 > config::HardMaxTranslationAccelerationMmS2 ||
        a.rotationalAccelMradS2 > config::HardMaxRotationalAccelerationMradS2 ||
        a.rotationalDecelMradS2 > config::HardMaxRotationalAccelerationMradS2 ||
        a.zeroCrossingHoldMs > 500 ||
        chassis.translationZeroThresholdMmS > 100 ||
        chassis.rotationZeroThresholdMradS > 200) return false;
#if ROBOT_ARM_CALIBRATION || ROBOT_ARM_CALIBRATED
    if (arm.firstLinkMm < 20 || arm.firstLinkMm > 300 ||
        arm.secondLinkMm < 20 || arm.secondLinkMm > 300 ||
        arm.shoulderBaseHeightMm > 500 || arm.gripperLengthOffsetMm > 200 ||
        arm.minimumReachMm >= arm.maximumReachMm || arm.maximumReachMm > 500 ||
        arm.minimumHeightMm >= arm.maximumHeightMm || arm.maximumHeightMm > 500 ||
        arm.cargoClearanceHeightMm < arm.minimumHeightMm ||
        arm.cargoClearanceHeightMm > arm.maximumHeightMm ||
        arm.presetReachMm < arm.minimumReachMm || arm.presetReachMm > arm.maximumReachMm ||
        arm.stowReachMm < arm.minimumReachMm || arm.stowReachMm > arm.maximumReachMm ||
        arm.presetHeightMm < arm.minimumHeightMm || arm.presetHeightMm > arm.maximumHeightMm ||
        arm.stowHeightMm < arm.minimumHeightMm || arm.stowHeightMm > arm.maximumHeightMm)
        return false;
#endif
    return assistDriveLimitPermille <= cargoDriveLimitPermille &&
           cargoDriveLimitPermille <= normalDriveLimitPermille &&
           normalDriveLimitPermille <= 1000;
}

bool RuntimeConfig::applyParameter(
    ParameterGroup group, uint8_t index, const uint8_t *data, uint8_t length,
    bool aggressiveAllowed
) {
    RuntimeConfig candidate = *this;
    switch (group) {
        case ParameterGroup::Servo:
            if (index >= 4 || length != 4) return false;
            candidate.servos[index] = {
                data[0], data[1], static_cast<int8_t>(data[2]), static_cast<int8_t>(data[3])
            };
            break;
        case ParameterGroup::UartOpenLoop:
            if (index >= 4 || length != 3) return false;
            candidate.uartOpenLoop[index] = {
                data[0], data[1], static_cast<int8_t>(data[2])
            };
            break;
        case ParameterGroup::ChassisSpeed:
            if (index != 0 || length != 10) return false;
            candidate.chassis.maximumForwardMmS = readI16(data);
            candidate.chassis.maximumReverseMmS = readI16(data + 2);
            candidate.chassis.maximumLateralMmS = readI16(data + 4);
            candidate.chassis.maximumYawMradS = readI16(data + 6);
            candidate.chassis.maximumWheelMmS = readU16(data + 8);
            break;
        case ParameterGroup::ChassisAcceleration:
            if (index == 0 && length == 16) {
                candidate.chassis.acceleration.forwardAccelMmS2 = readU16(data);
                candidate.chassis.acceleration.forwardDecelMmS2 = readU16(data + 2);
                candidate.chassis.acceleration.reverseAccelMmS2 = readU16(data + 4);
                candidate.chassis.acceleration.reverseDecelMmS2 = readU16(data + 6);
                candidate.chassis.acceleration.lateralAccelMmS2 = readU16(data + 8);
                candidate.chassis.acceleration.lateralDecelMmS2 = readU16(data + 10);
                candidate.chassis.acceleration.rotationalAccelMradS2 = readU16(data + 12);
                candidate.chassis.acceleration.rotationalDecelMradS2 = readU16(data + 14);
            } else if (index == 1 && length == 6) {
                candidate.chassis.acceleration.zeroCrossingHoldMs = readU16(data);
                candidate.chassis.translationZeroThresholdMmS = readU16(data + 2);
                candidate.chassis.rotationZeroThresholdMradS = readU16(data + 4);
            } else return false;
            break;
        case ParameterGroup::Encoder:
            if (index == 0 && length == 9) {
                candidate.encoder.wheelDiameterMm = readU16(data);
                candidate.encoder.countsPerRevolution = readU16(data + 2);
                candidate.encoder.wheelTrackMm = readU16(data + 4);
                candidate.encoder.wheelbaseMm = readU16(data + 6);
                candidate.encoder.semantics =
                    static_cast<EncoderSampleSemantics>(data[8]);
            } else if (index == 1 && length == 8) {
                for (uint8_t i = 0; i < 4; ++i)
                    candidate.encoder.channelMap[i] = static_cast<int8_t>(data[i]);
                for (uint8_t i = 0; i < 4; ++i)
                    candidate.encoder.signs[i] = static_cast<int8_t>(data[4 + i]);
            } else if (index == 2 && length == 8) {
                for (uint8_t i = 0; i < 4; ++i)
                    candidate.encoder.commandMap[i] = static_cast<int8_t>(data[i]);
                for (uint8_t i = 0; i < 4; ++i)
                    candidate.encoder.commandSigns[i] = static_cast<int8_t>(data[4 + i]);
            } else return false;
            break;
        case ParameterGroup::Sensor:
            if (index >= 6 || length != 2) return false;
            candidate.sensorOffsetMm[index] = readI16(data);
            break;
        case ParameterGroup::Assist:
            if (index != 0 || length != 6) return false;
            candidate.normalDriveLimitPermille = readU16(data);
            candidate.cargoDriveLimitPermille = readU16(data + 2);
            candidate.assistDriveLimitPermille = readU16(data + 4);
            break;
        case ParameterGroup::ResponseProfile:
            if (index == 0 && length == 1 &&
                data[0] <= static_cast<uint8_t>(ResponseProfile::Aggressive)) {
#if ROBOT_DRIVE_QUALIFICATION
                if (data[0] != static_cast<uint8_t>(ResponseProfile::Low)) return false;
#endif
                candidate.chassis.activeProfile = static_cast<ResponseProfile>(data[0]);
                if (candidate.chassis.activeProfile == ResponseProfile::Aggressive &&
                    !aggressiveAllowed) return false;
            } else if (index >= 1 && index <= 3 && length == 6) {
                candidate.responseProfiles[index - 1] = {
                    readU16(data), readU16(data + 2), readU16(data + 4)
                };
            } else return false;
            break;
        case ParameterGroup::ArmGeometry:
#if ROBOT_ARM_CALIBRATION
            if (index == 0 && length == 16) {
                candidate.arm.firstLinkMm = readU16(data);
                candidate.arm.secondLinkMm = readU16(data + 2);
                candidate.arm.shoulderBaseHeightMm = readU16(data + 4);
                candidate.arm.gripperLengthOffsetMm = readU16(data + 6);
                candidate.arm.minimumReachMm = readU16(data + 8);
                candidate.arm.maximumReachMm = readU16(data + 10);
                candidate.arm.minimumHeightMm = readU16(data + 12);
                candidate.arm.maximumHeightMm = readU16(data + 14);
            } else if (index == 1 && length == 10) {
                candidate.arm.cargoClearanceHeightMm = readU16(data);
                candidate.arm.presetReachMm = readU16(data + 2);
                candidate.arm.presetHeightMm = readU16(data + 4);
                candidate.arm.stowReachMm = readU16(data + 6);
                candidate.arm.stowHeightMm = readU16(data + 8);
            } else return false;
            break;
#else
            return false;
#endif
        default: return false;
    }
    if (!candidate.validate()) return false;
    candidate.revision = static_cast<uint16_t>(revision + 1U);
    *this = candidate;
    return true;
}

} // namespace robot
