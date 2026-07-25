#include "core/StackWatermark.h"

#include "app/BuildConfig.h"

#if ROBOT_CALIBRATION && defined(ARDUINO_ARCH_AVR)
#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>

extern uint8_t __heap_start;
extern void *__brkval;
#endif

namespace robot {

#if ROBOT_CALIBRATION && defined(ARDUINO_ARCH_AVR)
namespace {
constexpr uint8_t Canary = 0xA5;
constexpr uint8_t ConservativeGapBytes = 48;
uint8_t *canaryStart = 0;
uint16_t canaryLength = 0;
}

void beginStackWatermark() {
    const uint8_t savedStatus = SREG;
    cli();
    uint8_t *heapEnd = __brkval
        ? static_cast<uint8_t *>(__brkval) : &__heap_start;
    const uintptr_t stackPointer = SP;
    const uintptr_t paintEnd =
        stackPointer > ConservativeGapBytes
            ? stackPointer - ConservativeGapBytes : 0;
    canaryStart = heapEnd;
    canaryLength = paintEnd > reinterpret_cast<uintptr_t>(heapEnd)
        ? static_cast<uint16_t>(
              paintEnd - reinterpret_cast<uintptr_t>(heapEnd)
          )
        : 0;
    for (uint16_t index = 0; index < canaryLength; ++index)
        canaryStart[index] = Canary;
    SREG = savedStatus;
}

uint16_t minimumFreeStackBytes() {
    const uint8_t savedStatus = SREG;
    cli();
    uint16_t untouched = 0;
    while (untouched < canaryLength &&
           canaryStart[untouched] == Canary)
        ++untouched;
    SREG = savedStatus;
    return untouched;
}

#else

void beginStackWatermark() {}
uint16_t minimumFreeStackBytes() { return 0; }

#endif

} // namespace robot
