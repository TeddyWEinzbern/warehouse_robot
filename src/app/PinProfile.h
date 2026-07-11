#pragma once

#include <Arduino.h>

namespace robot {
namespace pins {

constexpr uint8_t BluetoothRx = A5;
constexpr uint8_t BluetoothTx = A4;

#if defined(ROBOT_BACKEND_L293D)
constexpr uint8_t ServoBase = A0;
constexpr uint8_t ServoShoulder = A1;
constexpr uint8_t ServoElbow = A2;
constexpr uint8_t ServoGripper = A3;
constexpr uint8_t SonarTrigger = 2;
constexpr uint8_t SonarEcho[] = {9, 10, 13};
constexpr uint8_t SonarDirection[] = {0, 1, 2};
constexpr uint8_t SonarSlot[] = {0, 0, 0};
#elif defined(ROBOT_BACKEND_UART)
constexpr uint8_t ServoBase = 3;
constexpr uint8_t ServoShoulder = 5;
constexpr uint8_t ServoElbow = 6;
constexpr uint8_t ServoGripper = 9;
constexpr uint8_t SonarTrigger = 2;
constexpr uint8_t SonarEcho[] = {4, 7, 8, 10, 11, 12};
constexpr uint8_t SonarDirection[] = {0, 0, 1, 1, 2, 2};
constexpr uint8_t SonarSlot[] = {0, 1, 0, 1, 0, 1};
#else
constexpr uint8_t ServoBase = 3;
constexpr uint8_t ServoShoulder = 5;
constexpr uint8_t ServoElbow = 6;
constexpr uint8_t ServoGripper = 9;
constexpr uint8_t SonarTrigger = 2;
constexpr uint8_t SonarEcho[] = {4};
constexpr uint8_t SonarDirection[] = {0};
constexpr uint8_t SonarSlot[] = {0};
#endif

constexpr uint8_t SonarCount = sizeof(SonarEcho) / sizeof(SonarEcho[0]);

static_assert(BluetoothRx != BluetoothTx, "Bluetooth RX/TX pins collide");
static_assert(ServoBase != ServoShoulder && ServoBase != ServoElbow &&
              ServoBase != ServoGripper && ServoShoulder != ServoElbow &&
              ServoShoulder != ServoGripper && ServoElbow != ServoGripper,
              "Servo pins must be unique");
static_assert(SonarTrigger != BluetoothRx && SonarTrigger != BluetoothTx &&
              SonarTrigger != ServoBase && SonarTrigger != ServoShoulder &&
              SonarTrigger != ServoElbow && SonarTrigger != ServoGripper,
              "Sonar trigger collides with a communication or servo pin");

} // namespace pins
} // namespace robot
