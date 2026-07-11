#pragma once

#include <Arduino.h>

#include "domain/RobotTypes.h"

namespace robot {

class CommunicationSubsystem {
  public:
    explicit CommunicationSubsystem(bool binaryMode);
    void poll(Stream &stream, uint32_t nowMs);
    const OperatorControlFrame &latest() const;

  private:
    bool binaryMode_;
    OperatorControlFrame latest_;
    uint16_t previousButtons_;
    uint8_t encoded_[24];
    uint8_t encodedLength_;
    char ascii_[48];
    uint8_t asciiLength_;
    void acceptFrame(OperatorControlFrame &frame, uint32_t nowMs);
    void finishBinary(uint32_t nowMs);
    void finishAscii(uint32_t nowMs);
    static uint8_t crc8(const uint8_t *data, uint8_t length);
    static uint8_t cobsDecode(
        const uint8_t *input, uint8_t length, uint8_t *output, uint8_t capacity
    );
};

} // namespace robot

