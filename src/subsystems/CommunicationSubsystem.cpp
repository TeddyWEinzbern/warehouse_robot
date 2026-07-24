#include "subsystems/CommunicationSubsystem.h"

#include <string.h>

namespace robot {
namespace {
constexpr uint8_t FastClassMask = 0xC0;
constexpr uint8_t FastSequenceMask = 0x3F;
constexpr uint8_t ControlClass = 0x40;
constexpr uint8_t EStopClass = 0x80;
constexpr uint8_t ControlRawLength = 9;
constexpr uint8_t EStopRawLength = 2;
constexpr uint8_t AllowedButtons = 0x3F;

int16_t readI16(const uint8_t *data) {
    return static_cast<int16_t>(
        static_cast<uint16_t>(data[0]) |
        (static_cast<uint16_t>(data[1]) << 8)
    );
}

uint16_t readU16(const uint8_t *data) {
    return static_cast<uint16_t>(data[0]) |
           (static_cast<uint16_t>(data[1]) << 8);
}
} // namespace

CommunicationSubsystem::CommunicationSubsystem()
    : latest_({}), requests_({0}), armMove_({}), jointReference_({}),
      driveCalibration_({}), calibrationRead_({}), previousButtons_(0),
      encoded_{}, raw_{}, transmit_{}, encodedLength_(0),
      transmitLength_(0), transmitOffset_(0), discarding_(false) {}

uint8_t CommunicationSubsystem::crc8(const uint8_t *data, uint8_t length) {
    uint8_t crc = 0;
    for (uint8_t index = 0; index < length; ++index) {
        crc ^= data[index];
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80U)
                ? static_cast<uint8_t>((crc << 1) ^ 0x07U)
                : static_cast<uint8_t>(crc << 1);
        }
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

uint8_t CommunicationSubsystem::cobsEncode(
    const uint8_t *input, uint8_t length, uint8_t *output, uint8_t capacity
) {
    if (capacity < 2) return 0;
    uint8_t read = 0;
    uint8_t written = 1;
    uint8_t codeIndex = 0;
    uint8_t code = 1;
    while (read < length) {
        if (input[read] == 0) {
            output[codeIndex] = code;
            codeIndex = written++;
            if (written > capacity) return 0;
            code = 1;
            ++read;
        } else {
            if (written >= capacity) return 0;
            output[written++] = input[read++];
            if (++code == 0xFF) {
                output[codeIndex] = code;
                codeIndex = written++;
                if (written > capacity) return 0;
                code = 1;
            }
        }
    }
    output[codeIndex] = code;
    return written;
}

int16_t CommunicationSubsystem::expandAxis(uint8_t value) {
    return static_cast<int16_t>(static_cast<int8_t>(value)) * 10;
}

int16_t CommunicationSubsystem::expandTernary(uint8_t code) {
    if (code == 1) return -1000;
    if (code == 2) return 1000;
    return 0;
}

void CommunicationSubsystem::acceptControl(
    const uint8_t *raw, uint32_t nowMs
) {
    for (uint8_t index = 1; index <= 5; ++index) {
        const int8_t axis = static_cast<int8_t>(raw[index]);
        if (axis < -100 || axis > 100) return;
    }
    const uint8_t discrete = raw[6];
    const uint8_t heightCode = discrete & 0x03U;
    const uint8_t gripperCode = (discrete >> 2) & 0x03U;
    if ((discrete & 0xF0U) != 0 || heightCode == 3 || gripperCode == 3 ||
        (raw[7] & ~AllowedButtons) != 0)
        return;

    OperatorControlFrame frame = {};
    frame.forward = expandAxis(raw[1]);
    frame.turn = expandAxis(raw[2]);
    frame.strafe = expandAxis(raw[3]);
    frame.armYaw = expandAxis(raw[4]);
    frame.armReach = expandAxis(raw[5]);
    frame.armHeight = expandTernary(heightCode);
    frame.gripper = static_cast<int8_t>(expandTernary(gripperCode) / 1000);
    frame.buttons = raw[7];
    frame.pressed = static_cast<uint16_t>(raw[7] & ~previousButtons_);
    frame.sequence = raw[0] & FastSequenceMask;
    frame.receivedAtMs = nowMs;
    frame.valid = true;
    previousButtons_ = raw[7];
    latest_ = frame;
}

void CommunicationSubsystem::acceptGeneric(uint8_t length) {
    if (length < 5 || raw_[0] != ProtocolVersion ||
        raw_[3] > MaximumPayload ||
        length != static_cast<uint8_t>(raw_[3] + 5U))
        return;

    const MessageType type = static_cast<MessageType>(raw_[1]);
    const uint8_t sequence = raw_[2];
    const uint8_t payloadLength = raw_[3];
    const uint8_t *payload = raw_ + 4;
    switch (type) {
        case MessageType::Hello:
            if (payloadLength == 0) requests_.flags |= RequestHello;
            break;
        case MessageType::Arm:
            if (payloadLength == 0) requests_.flags |= RequestArm;
            break;
        case MessageType::Disarm:
            if (payloadLength == 0) requests_.flags |= RequestDisarm;
            break;
        case MessageType::ClearEStop:
            if (payloadLength == 0) requests_.flags |= RequestClearEStop;
            break;
        case MessageType::ClearFault:
            if (payloadLength == 0) requests_.flags |= RequestClearFault;
            break;
        case MessageType::CalibrationArmMove:
            if (payloadLength == 2 && !armMove_.valid) {
                armMove_ = {true, payload[0], payload[1], sequence};
            }
            break;
        case MessageType::CalibrationSetJointReference:
            if (payloadLength == 5 && !jointReference_.valid) {
                jointReference_ = {
                    true, payload[0], payload[1], payload[2],
                    static_cast<int8_t>(payload[3]),
                    static_cast<int8_t>(payload[4]), sequence
                };
            }
            break;
        case MessageType::CalibrationDriveSpin:
            if (payloadLength == 6 && !driveCalibration_.valid) {
                driveCalibration_ = {
                    true, payload[0], payload[1], readI16(payload + 2),
                    readU16(payload + 4), sequence
                };
            }
            break;
        case MessageType::CalibrationReadDrive:
            if (payloadLength == 1 && payload[0] <= 1 &&
                !calibrationRead_.valid) {
                calibrationRead_ = {true, type, payload[0], sequence};
            }
            break;
        case MessageType::CalibrationReadSensor:
        case MessageType::CalibrationReadArm:
            if (payloadLength == 0 && !calibrationRead_.valid) {
                calibrationRead_ = {true, type, 0, sequence};
            }
            break;
#if ROBOT_CALIBRATION
        case MessageType::CalibrationReadSystem:
            if (payloadLength == 0 && !calibrationRead_.valid) {
                calibrationRead_ = {true, type, 0, sequence};
            }
            break;
#endif
        default:
            break;
    }
}

void CommunicationSubsystem::finishFrame(uint32_t nowMs) {
    const uint8_t length = cobsDecode(
        encoded_, encodedLength_, raw_, sizeof(raw_)
    );
    encodedLength_ = 0;
    if (length == 0 ||
        crc8(raw_, static_cast<uint8_t>(length - 1U)) != raw_[length - 1U])
        return;

    const uint8_t frameClass = raw_[0] & FastClassMask;
    if (frameClass == ControlClass) {
        if (length == ControlRawLength) acceptControl(raw_, nowMs);
        return;
    }
    if (frameClass == EStopClass) {
        if (length == EStopRawLength) requests_.flags |= RequestEStop;
        return;
    }
    acceptGeneric(length);
}

void CommunicationSubsystem::poll(Stream &stream, uint32_t nowMs) {
    uint8_t processed = 0;
    while (stream.available() > 0 && processed++ < 32) {
        const uint8_t value = static_cast<uint8_t>(stream.read());
        if (value == 0) {
            if (discarding_) {
                discarding_ = false;
                encodedLength_ = 0;
                continue;
            }
            if (encodedLength_ > 0) finishFrame(nowMs);
        } else if (discarding_) {
            continue;
        } else if (encodedLength_ < sizeof(encoded_)) {
            encoded_[encodedLength_++] = value;
        } else {
            encodedLength_ = 0;
            discarding_ = true;
        }
    }
}

bool CommunicationSubsystem::sendFrame(
    MessageType type, uint8_t sequence,
    const uint8_t *payload, uint8_t payloadLength
) {
    if (payloadLength > MaximumPayload || transmitLength_ != 0) return false;
    raw_[0] = ProtocolVersion;
    raw_[1] = static_cast<uint8_t>(type);
    raw_[2] = sequence;
    raw_[3] = payloadLength;
    if (payloadLength > 0 && payload != 0)
        memcpy(raw_ + 4, payload, payloadLength);
    raw_[4 + payloadLength] = crc8(
        raw_, static_cast<uint8_t>(4 + payloadLength)
    );
    const uint8_t encodedLength = cobsEncode(
        raw_, static_cast<uint8_t>(5 + payloadLength),
        transmit_, sizeof(transmit_) - 1
    );
    if (encodedLength == 0) return false;
    transmit_[encodedLength] = 0;
    transmitLength_ = static_cast<uint8_t>(encodedLength + 1U);
    transmitOffset_ = 0;
    return true;
}

bool CommunicationSubsystem::sendAck(
    uint8_t sequence, MessageType acknowledged
) {
    const uint8_t payload = static_cast<uint8_t>(acknowledged);
    return sendFrame(MessageType::Ack, sequence, &payload, 1);
}

bool CommunicationSubsystem::sendNack(
    uint8_t sequence, MessageType rejected, NackReason reason
) {
    const uint8_t payload[2] = {
        static_cast<uint8_t>(rejected), static_cast<uint8_t>(reason)
    };
    return sendFrame(MessageType::Nack, sequence, payload, sizeof(payload));
}

const OperatorControlFrame &CommunicationSubsystem::latest() const {
    return latest_;
}

ControlRequests CommunicationSubsystem::takeRequests() {
    const ControlRequests result = requests_;
    requests_.flags = 0;
    return result;
}

bool CommunicationSubsystem::takeArmMove(PendingArmMove &command) {
    if (!armMove_.valid) return false;
    command = armMove_;
    armMove_.valid = false;
    return true;
}

bool CommunicationSubsystem::takeJointReference(
    PendingJointReference &command
) {
    if (!jointReference_.valid) return false;
    command = jointReference_;
    jointReference_.valid = false;
    return true;
}

bool CommunicationSubsystem::takeDriveCalibration(
    PendingDriveCalibration &command
) {
    if (!driveCalibration_.valid) return false;
    command = driveCalibration_;
    driveCalibration_.valid = false;
    return true;
}

bool CommunicationSubsystem::takeCalibrationRead(
    PendingCalibrationRead &request
) {
    if (!calibrationRead_.valid) return false;
    request = calibrationRead_;
    calibrationRead_.valid = false;
    return true;
}

void CommunicationSubsystem::pumpTransmit(
    Stream &stream, uint8_t byteBudget
) {
    while (transmitLength_ != 0 && byteBudget-- > 0) {
        if (stream.write(transmit_[transmitOffset_]) != 1) return;
        ++transmitOffset_;
        if (transmitOffset_ >= transmitLength_) {
            transmitLength_ = 0;
            transmitOffset_ = 0;
        }
    }
}

bool CommunicationSubsystem::transmitIdle() const {
    return transmitLength_ == 0;
}

bool CommunicationSubsystem::receiveIdle() const {
    return encodedLength_ == 0 && !discarding_;
}

} // namespace robot
