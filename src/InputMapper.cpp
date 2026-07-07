#include "InputMapper.h"

RobotCommand InputMapper::map(const RobotCommand &command) const {
    // SerialProtocol already normalizes USB/Bluetooth text into one command
    // type. This layer exists so future button mappings or safety interlocks
    // have one place to translate raw inputs before RobotController acts on
    // them.
    return command;
}
