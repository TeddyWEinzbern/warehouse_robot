#pragma once

#include <stdint.h>

#ifndef ROBOT_DRIVER_ENABLED
#define ROBOT_DRIVER_ENABLED 0
#endif

#ifndef ROBOT_DRIVE_ENABLED
#define ROBOT_DRIVE_ENABLED 0
#endif
#ifndef ROBOT_ARM_ENABLED
#define ROBOT_ARM_ENABLED 0
#endif
#ifndef ROBOT_SENSOR_ENABLED
#define ROBOT_SENSOR_ENABLED 0
#endif
#ifndef ROBOT_CALIBRATION
#define ROBOT_CALIBRATION 0
#endif
#ifndef ROBOT_ARM_CALIBRATED
#define ROBOT_ARM_CALIBRATED 0
#endif
#ifndef ROBOT_DRIVE_CALIBRATED
#define ROBOT_DRIVE_CALIBRATED 0
#endif
#ifndef ROBOT_HOST_BAUD
#define ROBOT_HOST_BAUD 9600UL
#endif
#ifndef ROBOT_DRIVER_CONTROL_CLOSE
#define ROBOT_DRIVER_CONTROL_CLOSE 0
#endif
#ifndef ROBOT_DRIVER_CONTROL_OPEN
#define ROBOT_DRIVER_CONTROL_OPEN 0
#endif

#if ROBOT_DRIVER_ENABLED
#if (ROBOT_DRIVER_CONTROL_CLOSE + ROBOT_DRIVER_CONTROL_OPEN) != 1
#error "Select exactly one UART motor-controller mode"
#endif
#define ROBOT_HAS_ENCODER_FEEDBACK 1
#else
#if ROBOT_DRIVER_CONTROL_CLOSE || ROBOT_DRIVER_CONTROL_OPEN
#error "A motor-controller mode requires ROBOT_DRIVER_ENABLED=1"
#endif
#define ROBOT_HAS_ENCODER_FEEDBACK 0
#endif

#if ROBOT_DRIVE_ENABLED && !ROBOT_DRIVER_ENABLED
#error "ROBOT_DRIVE_ENABLED requires ROBOT_DRIVER_ENABLED"
#endif

#if ROBOT_HOST_BAUD != 9600UL && ROBOT_HOST_BAUD != 38400UL
#error "NeoSWSerial host link supports only 9600 or 38400 baud in this project"
#endif

namespace robot {
namespace config {

constexpr unsigned long BluetoothBaud = ROBOT_HOST_BAUD;
constexpr unsigned long MotorBoardBaud = 115200UL;

constexpr uint32_t CommandTimeoutMs = 300UL;
constexpr uint32_t NeutralQualificationMs = 500UL;
constexpr uint32_t MotorCommandPeriodUs = 20000UL;
constexpr uint32_t ZeroRepeatPeriodUs = 50000UL;
constexpr uint32_t EncoderQueryPeriodUs = 20000UL;
constexpr uint32_t ChassisRampPeriodUs = 10000UL;
constexpr uint32_t ServoPeriodUs = 20000UL;
constexpr uint32_t SonarGroupPeriodUs = 60000UL;
constexpr uint32_t CriticalStatusPeriodUs = 500000UL;
constexpr uint32_t HostControlPeriodUs = 33333UL;
// A normal typed safety command may immediately follow the 11-byte control
// frame. Ten milliseconds lets its seven 9600-baud bytes arrive before the
// Uno opens the reverse-direction response window.
constexpr uint32_t HostTransmitStartDelayUs = 10000UL;
constexpr uint32_t HostControlWireTimeUs =
    (11UL * 10UL * 1000000UL + BluetoothBaud - 1UL) / BluetoothBaud;
constexpr uint32_t HostTransmitWindowEndUs =
    HostControlPeriodUs - HostControlWireTimeUs - 2000UL;
constexpr uint32_t EncoderTotalPeriodUs = 500000UL;
constexpr uint32_t QueryTimeoutUs = 15000UL;
constexpr uint32_t FeedbackStaleMs = 100UL;
constexpr uint32_t SensorStaleMs = 300UL;
constexpr uint32_t AssistTimeoutMs = 5000UL;

constexpr uint32_t MaxLoopGapUs = 50000UL;
constexpr uint32_t MaxMotionDtUs = 50000UL;
constexpr uint32_t MotorLateThresholdUs = 10000UL;

constexpr bool DriverEnabled = ROBOT_DRIVER_ENABLED != 0;
constexpr bool DriveEnabled = ROBOT_DRIVE_ENABLED != 0;
constexpr bool ArmEnabled = ROBOT_ARM_ENABLED != 0;
constexpr bool SensorEnabled = ROBOT_SENSOR_ENABLED != 0;
constexpr bool DriveCalibrated = ROBOT_DRIVE_CALIBRATED != 0;
constexpr bool ArmCalibrated = ROBOT_ARM_CALIBRATED != 0;

static_assert(
    HostTransmitWindowEndUs > HostTransmitStartDelayUs,
    "Configured host baud leaves no safe response window"
);

constexpr float FirstLinkMm = 120.0F;
constexpr float SecondLinkMm = 120.0F;
constexpr float ShoulderBaseHeightMm = 55.0F;
constexpr float GripperLengthOffsetMm = 35.0F;
constexpr float GripperHeightOffsetMm = 15.0F;
constexpr float MinReachMm = 55.0F;
constexpr float MaxReachMm = 205.0F;
constexpr float MinHeightMm = 35.0F;
constexpr float MaxHeightMm = 190.0F;
constexpr float CargoClearanceHeightMm = 125.0F;
constexpr float PresetReachMm = 135.0F;
constexpr float PresetHeightMm = 105.0F;
constexpr float StowReachMm = 65.0F;
constexpr float StowHeightMm = 135.0F;

// Elbow drive four-bar: 40 mm horn and crank, 120 mm rod parallel to the
// 120 mm upper arm, crank fixed 20 degrees ahead of the forearm, links 10 mm
// wide. Rod-to-upper-arm clearance is 40*sin(fold + 20 deg), so fold 135 deg
// keeps >= 17 mm between link centrelines (bare contact is at 145.5 deg).
constexpr float ElbowFoldMinDegrees = 5.0F;
constexpr float ElbowFoldMaxDegrees = 135.0F;
// Extra fold allowance during calibration jog and for post-projection
// rounding; clearance at 138 deg is still >= 16 mm.
constexpr float ElbowFoldSlackDegrees = 3.0F;
constexpr float ServoClampToleranceDegrees = 1.5F;

constexpr float ArmYawDegreesPerSecond = 70.0F;
constexpr float ArmLinearMmPerSecond = 80.0F;
constexpr float GripperDegreesPerSecond = 60.0F;
constexpr float CalibrationDegreesPerSecond = 60.0F;

// Servo raw angles at the calibration anchor pose: upper arm vertical, horn
// at 20 degrees so the forearm is level, gripper open. Joint semantics:
// base offset = yaw - 90, shoulder offset = upper-arm angle from vertical,
// elbow offset = forearm angle from horizontal (absolute, four-bar drive).
constexpr uint8_t BaseZeroDegrees = 90;
constexpr uint8_t ShoulderZeroDegrees = 95;
constexpr uint8_t ElbowZeroDegrees = 95;
constexpr uint8_t GripperOpenDegrees = 80;
constexpr uint8_t GripperClosedDegrees = 25;

// Per-joint servo calibration measured with docs/calibration.md chapter 2.
// Joint order: 0 base, 1 shoulder, 2 elbow, 3 gripper. The `calibrate`
// REPL's `export` command prints this whole block ready to paste.
constexpr uint8_t ServoLowerDegrees[4] = {1, 40, 40, 5};
constexpr uint8_t ServoUpperDegrees[4] = {179, 180, 140, 175};
constexpr uint8_t ServoCenterDegrees[4] = {
    BaseZeroDegrees, ShoulderZeroDegrees, ElbowZeroDegrees, GripperOpenDegrees
};
constexpr int8_t ServoDirectionSign[4] = {1, -1, 1, 1};

// Per-wheel drive calibration measured with docs/calibration.md chapter 3.
// Logical wheel order everywhere: 0 front-left, 1 front-right, 2 rear-left,
// 3 rear-right. "Board channel" is the motor/encoder index (0..3 = M1..M4)
// on the UART motor driver board. The `calibrate` REPL's `export` command
// prints this whole block ready to paste.
constexpr int8_t MotorCommandMap[4] = {0, 1, 2, 3};
constexpr int8_t MotorCommandSign[4] = {1, 1, 1, 1};
constexpr int8_t EncoderChannelMap[4] = {0, 1, 2, 3};
constexpr int8_t EncoderDirectionSign[4] = {-1, 1, -1, 1};
constexpr uint16_t WheelDiameterMm = 60;
constexpr uint16_t EncoderCountsPerRevolution = 4680;
constexpr uint16_t WheelTrackMm = 160;
constexpr uint16_t WheelbaseMm = 170;

constexpr uint16_t NormalDriveLimitPermille = 1000;
constexpr uint16_t CargoDriveLimitPermille = 450;
constexpr uint16_t AssistDriveLimitPermille = 180;
constexpr uint16_t AssistCancelStickThreshold = 650;
constexpr uint16_t AssistManualBlendThreshold = 250;
constexpr uint16_t MinAssistDistanceMm = 120;
constexpr uint16_t AlignmentToleranceMm = 15;

// Caps for the calibration profile's motor spin commands (docs/calibration.md
// chapter 3): open-loop percent, closed-loop wheel speed, and run duration.
constexpr int16_t CalibrationSpinLimitPercent = 100;
constexpr int16_t CalibrationWheelLimitMmS = 200;
constexpr uint16_t CalibrationSpinMaxDurationMs = 10000;
constexpr uint16_t HardWheelLimitMmS = 1000;
constexpr int16_t HardMaxTranslationMmS = 1000;
constexpr int16_t HardMaxYawMradS = 3000;
constexpr uint16_t HardMaxTranslationAccelerationMmS2 = 3000;
constexpr uint16_t HardMaxRotationalAccelerationMradS2 = 8000;
constexpr uint16_t HardMaxWheelDiameterMm = 300;
constexpr uint16_t HardMaxCountsPerRevolution = 60000;
constexpr uint16_t HardMaxWheelGeometryMm = 1000;

enum class PresetPolicy : uint8_t { MinimumChange, ViaSafePose };
constexpr PresetPolicy ActivePresetPolicy = PresetPolicy::MinimumChange;

} // namespace config
} // namespace robot
