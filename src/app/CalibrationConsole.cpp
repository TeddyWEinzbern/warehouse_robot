#include "app/CalibrationConsole.h"

#include <stdlib.h>

namespace robot {

CalibrationConsole::CalibrationConsole() : length_(0) {}

void CalibrationConsole::execute(ArmSubsystem &arm) {
    line_[length_] = '\0';
    length_ = 0;
    if (line_[0] != 'J' || line_[1] != ':') return;
    char *cursor = line_ + 2;
    char *end = 0;
    const long joint = strtol(cursor, &end, 10);
    if (end == cursor || *end != ',') return;
    cursor = end + 1;
    const long degrees = strtol(cursor, &end, 10);
    if (end == cursor || *end != '\0' || joint < 0 || joint > 3 || degrees < 0 || degrees > 180) return;
    arm.setCalibrationJoint(static_cast<uint8_t>(joint), static_cast<uint8_t>(degrees));
}

void CalibrationConsole::poll(Stream &stream, ArmSubsystem &arm) {
    uint8_t processed = 0;
    while (stream.available() > 0 && processed++ < 32) {
        const char value = static_cast<char>(stream.read());
        if (value == '\r') continue;
        if (value == '\n') {
            if (length_ > 0) execute(arm);
        } else if (length_ < sizeof(line_) - 1) line_[length_++] = value;
        else length_ = 0;
    }
}

} // namespace robot
