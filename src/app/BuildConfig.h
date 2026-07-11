#pragma once

#if (defined(ROBOT_BACKEND_L293D) + defined(ROBOT_BACKEND_UART) +             \
     defined(ROBOT_BACKEND_NONE)) != 1
#error "Select exactly one robot drive backend"
#endif

#ifndef ROBOT_ARM_CALIBRATION
#define ROBOT_ARM_CALIBRATION 0
#endif

#if defined(ROBOT_BACKEND_UART)
#define ROBOT_HAS_ENCODER_FEEDBACK 1
#define ROBOT_HAS_CLOSED_LOOP_SPEED 1
#else
#define ROBOT_HAS_ENCODER_FEEDBACK 0
#define ROBOT_HAS_CLOSED_LOOP_SPEED 0
#endif

#if defined(ROBOT_REQUIRE_ENCODERS) && !ROBOT_HAS_ENCODER_FEEDBACK
#error "This build requires encoder feedback, but the selected backend cannot provide it"
#endif

namespace robot {
namespace config {

constexpr unsigned long UsbBaud = 115200UL;
constexpr unsigned long BluetoothBaud = 9600UL;
constexpr unsigned long MotorBoardBaud = 115200UL;
constexpr unsigned long CommandTimeoutMs = 300UL;
constexpr unsigned long SensorStaleMs = 250UL;
constexpr unsigned long AssistTimeoutMs = 5000UL;

constexpr bool ArmCalibrated = false;
constexpr float FirstLinkMm = 110.0F;
constexpr float SecondLinkMm = 110.0F;
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
constexpr float ManualYawDegreesPerSecond = 70.0F;
constexpr float ManualLinearMmPerSecond = 80.0F;

constexpr uint8_t BaseZeroDegrees = 90;
constexpr uint8_t ShoulderZeroDegrees = 90;
constexpr uint8_t ElbowZeroDegrees = 10;
constexpr uint8_t GripperOpenDegrees = 80;
constexpr uint8_t GripperClosedDegrees = 25;

constexpr uint16_t NormalDriveLimit = 1000;
constexpr uint16_t CargoDriveLimit = 450;
constexpr uint16_t AssistDriveLimit = 180;
constexpr uint16_t AssistCancelStickThreshold = 650;
constexpr uint16_t AssistManualBlendThreshold = 250;
constexpr uint16_t MinAssistDistanceMm = 120;
constexpr uint16_t AlignmentToleranceMm = 15;

enum class PresetPolicy { MinimumChange, ViaSafePose };
constexpr PresetPolicy ActivePresetPolicy = PresetPolicy::MinimumChange;

} // namespace config
} // namespace robot
