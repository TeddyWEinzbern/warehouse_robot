#include "core/MotorBoardProtocol.h"

#include <limits.h>

namespace robot {

MotorBoardFrameParser::MotorBoardFrameParser()
    : buffer_{}, overflows_(0), length_(0), collecting_(false) {}

MotorBoardFeedResult MotorBoardFrameParser::feed(char value) {
    if (value == '$') {
        collecting_ = true;
        length_ = 0;
        buffer_[length_++] = value;
        return MotorBoardFeedResult::None;
    }
    if (!collecting_) return MotorBoardFeedResult::None;
    if (value == '!') {
        if (length_ < sizeof(buffer_) - 1U) {
            buffer_[length_++] = value;
            buffer_[length_] = '\0';
            collecting_ = false;
            return MotorBoardFeedResult::Complete;
        }
    } else if (length_ < sizeof(buffer_) - 1U) {
        buffer_[length_++] = value;
        return MotorBoardFeedResult::None;
    }
    collecting_ = false;
    length_ = 0;
    if (overflows_ != 65535U) ++overflows_;
    return MotorBoardFeedResult::Overflow;
}

const char *MotorBoardFrameParser::frame() const { return buffer_; }
uint16_t MotorBoardFrameParser::overflows() const { return overflows_; }

bool MotorBoardFrameParser::parseInteger(
    const char *&cursor, int32_t &result, int32_t minimum, int32_t maximum,
    char delimiter
) {
    bool negative = false;
    if (*cursor == '-') { negative = true; ++cursor; }
    if (*cursor < '0' || *cursor > '9') return false;
    const uint32_t magnitudeLimit = negative ? 2147483648UL : 2147483647UL;
    uint32_t magnitude = 0;
    do {
        const uint8_t digit = static_cast<uint8_t>(*cursor - '0');
        if (magnitude > (magnitudeLimit - digit) / 10UL) return false;
        magnitude = magnitude * 10UL + digit;
        ++cursor;
    } while (*cursor >= '0' && *cursor <= '9');
    if (*cursor != delimiter) return false;
    result = negative
        ? (magnitude == 2147483648UL
               ? INT32_MIN : -static_cast<int32_t>(magnitude))
        : static_cast<int32_t>(magnitude);
    if (result < minimum || result > maximum) return false;
    ++cursor;
    return true;
}

bool MotorBoardFrameParser::parseFour(
    const char *cursor, int32_t *values, int32_t minimum, int32_t maximum
) {
    for (uint8_t index = 0; index < 4; ++index) {
        if (!parseInteger(
                cursor, values[index], minimum, maximum,
                index == 3 ? '!' : ','
            )) return false;
    }
    return *cursor == '\0';
}

} // namespace robot
