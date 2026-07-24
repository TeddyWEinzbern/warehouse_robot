#pragma once

#include <stdint.h>

namespace robot {

enum class MotorBoardFeedResult : uint8_t { None, Complete, Overflow };

class MotorBoardFrameParser {
  public:
    MotorBoardFrameParser();
    MotorBoardFeedResult feed(char value);
    const char *frame() const;
    uint16_t overflows() const;
    static bool parseFour(
        const char *cursor,
        int32_t *values,
        int32_t minimum,
        int32_t maximum
    );

  private:
    // The longest supported reply is the 73-byte encoder-total frame with
    // four signed int32 values. Keep a small terminator margin, not a generic
    // debug-line buffer.
    char buffer_[80];
    uint16_t overflows_;
    uint8_t length_;
    bool collecting_;
    static bool parseInteger(
        const char *&cursor,
        int32_t &result,
        int32_t minimum,
        int32_t maximum,
        char delimiter
    );
};

} // namespace robot
