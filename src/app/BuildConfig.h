#pragma once

#include <stdint.h>

#if (defined(ROBOT_BACKEND_UART) + defined(ROBOT_BACKEND_NONE)) != 1
#error "Select exactly one robot drive backend"
#endif

#ifndef ROBOT_ARM_CALIBRATION
#define ROBOT_ARM_CALIBRATION 0
#endif
#ifndef ROBOT_SAFE_IDLE
#define ROBOT_SAFE_IDLE 0
#endif
#ifndef ROBOT_DRIVE_QUALIFICATION
#define ROBOT_DRIVE_QUALIFICATION 0
#endif
#ifndef ROBOT_DRIVE_CALIBRATION_QUALIFIED
#define ROBOT_DRIVE_CALIBRATION_QUALIFIED 0
#endif
#ifndef ROBOT_ARM_CALIBRATED
#define ROBOT_ARM_CALIBRATED 0
#endif
#ifndef ROBOT_HOST_BAUD
#define ROBOT_HOST_BAUD 38400UL
#endif

#if defined(ROBOT_BACKEND_UART)
#if (defined(ROBOT_UART_CLOSED_LOOP) + defined(ROBOT_UART_OPEN_LOOP)) != 1
#error "Select exactly one UART motor-controller mode"
#endif
#define ROBOT_HAS_ENCODER_FEEDBACK 1
#else
#define ROBOT_HAS_ENCODER_FEEDBACK 0
#endif

namespace robot {
namespace config {

constexpr unsigned long UsbBaud = 115200UL;
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
constexpr uint32_t TelemetryPeriodUs =
    ROBOT_HOST_BAUD == 9600UL ? 200000UL : 100000UL;
constexpr uint32_t EncoderTotalPeriodUs = 500000UL;
constexpr uint32_t BatteryPeriodUs = 1000000UL;
constexpr uint32_t QueryTimeoutUs = 15000UL;
constexpr uint32_t FeedbackStaleMs = 100UL;
constexpr uint32_t SensorStaleMs = 300UL;
constexpr uint32_t AssistTimeoutMs = 5000UL;

constexpr uint32_t MaxLoopGapUs = 50000UL;
constexpr uint32_t MaxMotionDtUs = 50000UL;
constexpr uint32_t MotorLateThresholdUs = 10000UL;

constexpr bool DriveCalibrationQualified = ROBOT_DRIVE_CALIBRATION_QUALIFIED != 0;
constexpr bool ArmCalibrated = ROBOT_ARM_CALIBRATED != 0;

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

// Per-joint servo calibration measured with docs/arm-calibration.md.
// Joint order: 0 base, 1 shoulder, 2 elbow, 3 gripper. The `calibrate`
// REPL's `export` command prints this whole block ready to paste.
constexpr uint8_t ServoLowerDegrees[4] = {1, 40, 40, 5};
constexpr uint8_t ServoUpperDegrees[4] = {179, 180, 140, 175};
constexpr uint8_t ServoCenterDegrees[4] = {
    BaseZeroDegrees, ShoulderZeroDegrees, ElbowZeroDegrees, GripperOpenDegrees
};
constexpr int8_t ServoDirectionSign[4] = {1, -1, 1, 1};

constexpr uint16_t NormalDriveLimitPermille = 1000;
constexpr uint16_t CargoDriveLimitPermille = 450;
constexpr uint16_t AssistDriveLimitPermille = 180;
constexpr uint16_t AssistCancelStickThreshold = 650;
constexpr uint16_t AssistManualBlendThreshold = 250;
constexpr uint16_t MinAssistDistanceMm = 120;
constexpr uint16_t AlignmentToleranceMm = 15;

constexpr uint16_t QualificationWheelLimitMmS = 200;
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
