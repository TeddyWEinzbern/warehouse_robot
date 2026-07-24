#pragma once

#include <Arduino.h>
#include <stddef.h>

namespace robot {
namespace pins {

constexpr uint8_t BluetoothRx = A5;
constexpr uint8_t BluetoothTx = A4;

#if ROBOT_DRIVER_ENABLED
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

namespace detail {
constexpr bool fixedPinConflict(uint8_t pin) {
    return pin == 0 || pin == 1 ||
           pin == BluetoothRx || pin == BluetoothTx ||
           pin == ServoBase || pin == ServoShoulder ||
           pin == ServoElbow || pin == ServoGripper;
}

template <size_t N>
constexpr bool avoidsFixedPins(
    const uint8_t (&values)[N], size_t index = 0
) {
    return index == N
        ? true
        : (!fixedPinConflict(values[index]) &&
           avoidsFixedPins(values, index + 1));
}

template <size_t N>
constexpr bool absentAfter(
    const uint8_t (&values)[N], uint8_t value, size_t index
) {
    return index == N
        ? true
        : (values[index] != value &&
           absentAfter(values, value, index + 1));
}

template <size_t N>
constexpr bool uniquePins(
    const uint8_t (&values)[N], size_t index = 0
) {
    return index == N
        ? true
        : (absentAfter(values, values[index], index + 1) &&
           uniquePins(values, index + 1));
}

template <size_t N, size_t M>
constexpr bool disjointPins(
    const uint8_t (&first)[N], const uint8_t (&second)[M],
    size_t index = 0
) {
    return index == N
        ? true
        : (absentAfter(second, first[index], 0) &&
           disjointPins(first, second, index + 1));
}

template <size_t N>
constexpr bool valuesBelow(
    const uint8_t (&values)[N], uint8_t limit, size_t index = 0
) {
    return index == N
        ? true
        : (values[index] < limit &&
           valuesBelow(values, limit, index + 1));
}

template <size_t N>
constexpr bool pairAbsentAfter(
    const uint8_t (&first)[N], const uint8_t (&second)[N],
    uint8_t firstValue, uint8_t secondValue, size_t index
) {
    return index == N
        ? true
        : ((first[index] != firstValue || second[index] != secondValue) &&
           pairAbsentAfter(
               first, second, firstValue, secondValue, index + 1
           ));
}

template <size_t N>
constexpr bool uniquePairs(
    const uint8_t (&first)[N], const uint8_t (&second)[N],
    size_t index = 0
) {
    return index == N
        ? true
        : (pairAbsentAfter(
               first, second, first[index], second[index], index + 1
           ) && uniquePairs(first, second, index + 1));
}
} // namespace detail

static_assert(BluetoothRx != BluetoothTx, "Bluetooth RX/TX pins collide");
static_assert(ServoBase != ServoShoulder && ServoBase != ServoElbow &&
              ServoBase != ServoGripper && ServoShoulder != ServoElbow &&
              ServoShoulder != ServoGripper && ServoElbow != ServoGripper,
              "Servo pins must be unique");
static_assert(SonarCount <= 6, "Sensor snapshot supports at most six echoes");
static_assert(
    sizeof(SonarDirection) == sizeof(SonarEcho) &&
    sizeof(SonarSlot) == sizeof(SonarEcho) &&
    sizeof(SonarGroup) == sizeof(SonarEcho),
    "Every sonar echo needs direction, slot, and group entries"
);
static_assert(
    detail::avoidsFixedPins(SonarTrigger) &&
    detail::avoidsFixedPins(SonarEcho),
    "Sonar pin collides with motor UART, HC-06, or a servo"
);
static_assert(
    detail::uniquePins(SonarTrigger) &&
    detail::uniquePins(SonarEcho) &&
    detail::disjointPins(SonarTrigger, SonarEcho),
    "Sonar trigger and echo pins must be unique"
);
static_assert(
    detail::valuesBelow(SonarDirection, 3) &&
    detail::valuesBelow(SonarSlot, 2) &&
    detail::valuesBelow(SonarGroup, SonarGroupCount),
    "Sonar direction, slot, or group index is out of range"
);
static_assert(
    detail::uniquePairs(SonarDirection, SonarSlot),
    "Each sonar direction/slot pair must be unique"
);

} // namespace pins
} // namespace robot
