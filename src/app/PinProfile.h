#pragma once

#include <Arduino.h>

namespace robot {
namespace pins {

constexpr uint8_t BluetoothRx = A5;
constexpr uint8_t BluetoothTx = A4;

#if defined(ROBOT_BACKEND_UART)
constexpr uint8_t ServoBase = 3;
constexpr uint8_t ServoShoulder = 5;
constexpr uint8_t ServoElbow = 6;
constexpr uint8_t ServoGripper = 9;
// Each 60 ms slot fires one front/left/right group. The second trigger requires
// the documented D13 wiring change before sonar is enabled on UART profiles.
constexpr uint8_t SonarTrigger[] = {2, 13};
constexpr uint8_t SonarEcho[] = {4, 8, 11, 7, 10, 12};
constexpr uint8_t SonarDirection[] = {0, 1, 2, 0, 1, 2};
constexpr uint8_t SonarSlot[] = {0, 0, 0, 1, 1, 1};
constexpr uint8_t SonarGroup[] = {0, 0, 0, 1, 1, 1};
#else
constexpr uint8_t ServoBase = 3;
constexpr uint8_t ServoShoulder = 5;
constexpr uint8_t ServoElbow = 6;
constexpr uint8_t ServoGripper = 9;
constexpr uint8_t SonarTrigger[] = {2};
constexpr uint8_t SonarEcho[] = {4};
constexpr uint8_t SonarDirection[] = {0};
constexpr uint8_t SonarSlot[] = {0};
constexpr uint8_t SonarGroup[] = {0};
#endif

constexpr uint8_t SonarCount = sizeof(SonarEcho) / sizeof(SonarEcho[0]);
constexpr uint8_t SonarGroupCount = sizeof(SonarTrigger) / sizeof(SonarTrigger[0]);

static_assert(BluetoothRx != BluetoothTx, "Bluetooth RX/TX pins collide");
static_assert(ServoBase != ServoShoulder && ServoBase != ServoElbow &&
              ServoBase != ServoGripper && ServoShoulder != ServoElbow &&
              ServoShoulder != ServoGripper && ServoElbow != ServoGripper,
              "Servo pins must be unique");
static_assert(SonarTrigger[0] != BluetoothRx && SonarTrigger[0] != BluetoothTx &&
              SonarTrigger[0] != ServoBase && SonarTrigger[0] != ServoShoulder &&
              SonarTrigger[0] != ServoElbow && SonarTrigger[0] != ServoGripper,
              "Sonar trigger collides with a communication or servo pin");

} // namespace pins
} // namespace robot
