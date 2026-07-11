#include "subsystems/CommunicationSubsystem.h"

#include <stdlib.h>
#include <string.h>

namespace robot {
namespace {
int16_t readI16(const uint8_t *data) {
    return static_cast<int16_t>(
        static_cast<uint16_t>(data[0]) |
        (static_cast<uint16_t>(data[1]) << 8)
    );
}

bool parseAxis(char *&cursor, char separator, int16_t &value) {
    char *end = 0;
    const long parsed = strtol(cursor, &end, 10);
    if (end == cursor || *end != separator) return false;
    long limited = parsed;
    if (limited < -1000) limited = -1000;
    if (limited > 1000) limited = 1000;
    value = static_cast<int16_t>(limited);
    cursor = end + 1;
    return true;
}

bool parseFinalAxis(char *cursor, int16_t &value) {
    char *end = 0;
    long parsed = strtol(cursor, &end, 10);
    if (end == cursor || *end != '\0') return false;
    if (parsed < -1000) parsed = -1000;
    if (parsed > 1000) parsed = 1000;
    value = static_cast<int16_t>(parsed);
    return true;
}
}

CommunicationSubsystem::CommunicationSubsystem(bool binaryMode)
    : binaryMode_(binaryMode), latest_({}),
      previousButtons_(0), encodedLength_(0), asciiLength_(0) {}

uint8_t CommunicationSubsystem::crc8(const uint8_t *data, uint8_t length) {
    uint8_t crc = 0;
    for (uint8_t index = 0; index < length; ++index) {
        crc ^= data[index];
        for (uint8_t bit = 0; bit < 8; ++bit)
            crc = (crc & 0x80U) ? static_cast<uint8_t>((crc << 1) ^ 0x07U)
                                : static_cast<uint8_t>(crc << 1);
    }
    return crc;
}

uint8_t CommunicationSubsystem::cobsDecode(
    const uint8_t *input, uint8_t length, uint8_t *output, uint8_t capacity
) {
    uint8_t read = 0;
    uint8_t written = 0;
    while (read < length) {
        const uint8_t code = input[read++];
        if (code == 0) return 0;
        for (uint8_t index = 1; index < code; ++index) {
            if (read >= length || written >= capacity) return 0;
            output[written++] = input[read++];
        }
        if (code != 0xFF && read < length) {
            if (written >= capacity) return 0;
            output[written++] = 0;
        }
    }
    return written;
}

void CommunicationSubsystem::acceptFrame(OperatorControlFrame &frame, uint32_t nowMs) {
    frame.pressed = static_cast<uint16_t>(frame.buttons & ~previousButtons_);
    previousButtons_ = frame.buttons;
    frame.receivedAtMs = nowMs;
    frame.valid = true;
    latest_ = frame;
}

void CommunicationSubsystem::finishBinary(uint32_t nowMs) {
    uint8_t raw[20];
    const uint8_t length = cobsDecode(encoded_, encodedLength_, raw, sizeof(raw));
    encodedLength_ = 0;
    // version, sequence, six int16 axes, gripper, buttons uint16, crc
    if (length != 18 || raw[0] != 1 || crc8(raw, 17) != raw[17]) return;
    OperatorControlFrame frame = {};
    frame.sequence = raw[1];
    frame.forward = readI16(raw + 2);
    frame.turn = readI16(raw + 4);
    frame.strafe = readI16(raw + 6);
    frame.armYaw = readI16(raw + 8);
    frame.armReach = readI16(raw + 10);
    frame.armHeight = readI16(raw + 12);
    frame.gripper = static_cast<int8_t>(raw[14]);
    frame.buttons = static_cast<uint16_t>(raw[15]) |
                    (static_cast<uint16_t>(raw[16]) << 8);
    frame.directWheels = false;
    acceptFrame(frame, nowMs);
}

void CommunicationSubsystem::finishAscii(uint32_t nowMs) {
    ascii_[asciiLength_] = '\0';
    asciiLength_ = 0;
    if (ascii_[1] != ':') return;
    OperatorControlFrame frame = latest_;
    char *cursor = ascii_ + 2;
    if (ascii_[0] == 'C') {
        if (!parseAxis(cursor, ',', frame.forward) ||
            !parseAxis(cursor, ',', frame.turn) ||
            !parseFinalAxis(cursor, frame.strafe)) return;
        frame.directWheels = false;
    } else if (ascii_[0] == 'W') {
        if (!parseAxis(cursor, ',', frame.wheelFrontLeft) ||
            !parseAxis(cursor, ',', frame.wheelFrontRight) ||
            !parseAxis(cursor, ',', frame.wheelRearLeft) ||
            !parseFinalAxis(cursor, frame.wheelRearRight)) return;
        frame.directWheels = true;
        frame.forward = frame.turn = frame.strafe = 0;
    } else if (ascii_[0] == 'M') {
        int16_t motor = 0;
        int16_t speed = 0;
        if (!parseAxis(cursor, ',', motor) || !parseFinalAxis(cursor, speed) ||
            motor < 1 || motor > 4) return;
        frame.wheelFrontLeft = motor == 1 ? speed : 0;
        frame.wheelFrontRight = motor == 2 ? speed : 0;
        frame.wheelRearLeft = motor == 3 ? speed : 0;
        frame.wheelRearRight = motor == 4 ? speed : 0;
        frame.directWheels = true;
        frame.forward = frame.turn = frame.strafe = 0;
    } else return;
    frame.armYaw = frame.armReach = frame.armHeight = 0;
    frame.gripper = 0;
    frame.buttons = 0;
    frame.sequence++;
    acceptFrame(frame, nowMs);
}

void CommunicationSubsystem::poll(Stream &stream, uint32_t nowMs) {
    while (stream.available() > 0) {
        const uint8_t value = static_cast<uint8_t>(stream.read());
        if (binaryMode_) {
            if (value == 0) {
                if (encodedLength_ > 0) finishBinary(nowMs);
            } else if (encodedLength_ < sizeof(encoded_)) encoded_[encodedLength_++] = value;
            else encodedLength_ = 0;
        } else {
            if (value == '\r') continue;
            if (value == '\n') {
                if (asciiLength_ > 0) finishAscii(nowMs);
            } else if (asciiLength_ < sizeof(ascii_) - 1) ascii_[asciiLength_++] = static_cast<char>(value);
            else asciiLength_ = 0;
        }
    }
}

const OperatorControlFrame &CommunicationSubsystem::latest() const { return latest_; }

} // namespace robot
