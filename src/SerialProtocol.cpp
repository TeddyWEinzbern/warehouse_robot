#include "SerialProtocol.h"

#include <stdlib.h>
#include <string.h>

namespace {

bool parseInteger(char *&cursor, char separator, long &value) {
    char *end = nullptr;
    value = strtol(cursor, &end, 10);
    if (end == cursor || *end != separator) {
        return false;
    }
    cursor = end + 1;
    return true;
}

bool parseFinalInteger(char *&cursor, long &value) {
    char *end = nullptr;
    value = strtol(cursor, &end, 10);
    if (end == cursor || *end != '\0') {
        return false;
    }
    cursor = end;
    return true;
}

} // namespace

namespace SerialProtocol {

CommandReader::CommandReader() : length_(0), overflowed_(false) {
    buffer_[0] = '\0';
    lastLine_[0] = '\0';
}

void CommandReader::reset() {
    length_ = 0;
    overflowed_ = false;
    buffer_[0] = '\0';
}

const char *CommandReader::lastLine() const {
    return lastLine_;
}

ReadResult CommandReader::read(
    Stream &stream, RobotCommand &command, unsigned long &bytesRead
) {
    while (stream.available() > 0) {
        const char incoming = static_cast<char>(stream.read());
        bytesRead++;

        if (incoming == '\r') {
            continue;
        }

        if (incoming == '\n') {
            if (overflowed_) {
                reset();
                return ReadResult::None;
            }
            return finishLine(command);
        }

        if (overflowed_) {
            continue;
        }

        if (length_ < Config::COMMAND_BUFFER_SIZE - 1) {
            buffer_[length_++] = incoming;
            buffer_[length_] = '\0';
        } else {
            overflowed_ = true;
            length_ = 0;
            buffer_[0] = '\0';
            lastLine_[0] = '\0';
            return ReadResult::Overflow;
        }
    }

    return ReadResult::None;
}

ReadResult CommandReader::finishLine(RobotCommand &command) {
    if (length_ == 0) {
        reset();
        return ReadResult::None;
    }

    buffer_[length_] = '\0';
    strncpy(lastLine_, buffer_, Config::COMMAND_BUFFER_SIZE);
    lastLine_[Config::COMMAND_BUFFER_SIZE - 1] = '\0';

    const bool ok = parseCommand(buffer_, command);
    reset();
    return ok ? ReadResult::ValidCommand : ReadResult::InvalidCommand;
}

int clampAxis(long value) {
    if (value < -1000) {
        return -1000;
    }
    if (value > 1000) {
        return 1000;
    }
    return static_cast<int>(value);
}

bool parseCommand(char *line, RobotCommand &command) {
    command.type = RobotCommandType::None;

    if (line[0] == 'C' && line[1] == ':') {
        char *cursor = line + 2;
        long forward = 0;
        long turn = 0;
        long strafe = 0;

        if (!parseInteger(cursor, ',', forward) ||
            !parseInteger(cursor, ',', turn) ||
            !parseFinalInteger(cursor, strafe)) {
            return false;
        }

        command.type = RobotCommandType::Drive;
        command.drive.forward = clampAxis(forward);
        command.drive.turn = clampAxis(turn);
        command.drive.strafe = clampAxis(strafe);
        return true;
    }

    if (line[0] == 'W' && line[1] == ':') {
        char *cursor = line + 2;
        long frontLeft = 0;
        long frontRight = 0;
        long rearLeft = 0;
        long rearRight = 0;

        if (!parseInteger(cursor, ',', frontLeft) ||
            !parseInteger(cursor, ',', frontRight) ||
            !parseInteger(cursor, ',', rearLeft) ||
            !parseFinalInteger(cursor, rearRight)) {
            return false;
        }

        command.type = RobotCommandType::WheelSpeeds;
        command.wheels.frontLeft = clampAxis(frontLeft);
        command.wheels.frontRight = clampAxis(frontRight);
        command.wheels.rearLeft = clampAxis(rearLeft);
        command.wheels.rearRight = clampAxis(rearRight);
        return true;
    }

    if (line[0] == 'M' && line[1] == ':') {
        char *cursor = line + 2;
        long motorIndex = 0;
        long speed = 0;

        if (!parseInteger(cursor, ',', motorIndex) ||
            !parseFinalInteger(cursor, speed) || motorIndex < 1 ||
            motorIndex > 4) {
            return false;
        }

        command.type = RobotCommandType::SingleMotor;
        command.singleMotor.motorIndex = static_cast<uint8_t>(motorIndex);
        command.singleMotor.speed = clampAxis(speed);
        return true;
    }

    return false;
}

} // namespace SerialProtocol
