#pragma once

#include <Arduino.h>

#include "Config.h"
#include "State.h"

namespace SerialProtocol {

enum class ReadResult : uint8_t {
    None,
    ValidCommand,
    InvalidCommand,
    Overflow,
};

class CommandReader {
  public:
    CommandReader();

    ReadResult read(
        Stream &stream, RobotCommand &command, unsigned long &bytesRead
    );
    const char *lastLine() const;
    void reset();

  private:
    char buffer_[Config::COMMAND_BUFFER_SIZE];
    char lastLine_[Config::COMMAND_BUFFER_SIZE];
    size_t length_;
    bool overflowed_;

    ReadResult finishLine(RobotCommand &command);
};

bool parseCommand(char *line, RobotCommand &command);
int clampAxis(long value);

} // namespace SerialProtocol
