#include "subsystems/CommunicationSubsystem.h"

#include <string.h>

namespace robot {
namespace {
constexpr uint8_t ProtocolVersion = 2;
constexpr uint8_t MaximumPayload = 27;

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

void saturatingIncrement(uint16_t &value) {
    if (value != 65535U) ++value;
}

bool validAxis(int16_t value) { return value >= -1000 && value <= 1000; }
} // namespace

CommunicationSubsystem::CommunicationSubsystem()
    : latest_({}), requests_({0}), parameter_({}), calibration_({}),
      driveCalibration_({}),
      previousButtons_(0), rxOverflows_(0), malformedFrames_(0),
      snapshotSequence_(0), encodedLength_(0), transmit_{},
      transmitLength_(0), transmitOffset_(0), txDrops_(0),
      snapshotRequested_(false) {}

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

void CommunicationSubsystem::markMalformed() {
    saturatingIncrement(malformedFrames_);
}

void CommunicationSubsystem::acceptFrame(
    OperatorControlFrame &frame, uint32_t nowMs
) {
    frame.pressed = static_cast<uint16_t>(frame.buttons & ~previousButtons_);
    previousButtons_ = frame.buttons;
    frame.receivedAtMs = nowMs;
    frame.valid = true;
    latest_ = frame;
}

void CommunicationSubsystem::finishFrame(uint32_t nowMs) {
    uint8_t raw[32];
    const uint8_t length = cobsDecode(encoded_, encodedLength_, raw, sizeof(raw));
    encodedLength_ = 0;
    if (length < 5 || raw[0] != ProtocolVersion || raw[3] > MaximumPayload ||
        length != static_cast<uint8_t>(raw[3] + 5U) ||
        crc8(raw, static_cast<uint8_t>(length - 1U)) != raw[length - 1U]) {
        markMalformed();
        return;
    }
    const MessageType type = static_cast<MessageType>(raw[1]);
    const uint8_t sequence = raw[2];
    const uint8_t payloadLength = raw[3];
    const uint8_t *payload = raw + 4;
    switch (type) {
        case MessageType::Control: {
            if (payloadLength != 16) { markMalformed(); return; }
            OperatorControlFrame frame = {};
            frame.forward = readI16(payload);
            frame.turn = readI16(payload + 2);
            frame.strafe = readI16(payload + 4);
            frame.armYaw = readI16(payload + 6);
            frame.armReach = readI16(payload + 8);
            frame.armHeight = readI16(payload + 10);
            frame.gripper = static_cast<int8_t>(payload[12]);
            frame.buttons = readU16(payload + 13);
            frame.controlFlags = payload[15];
            frame.sequence = sequence;
            if (!validAxis(frame.forward) || !validAxis(frame.turn) ||
                !validAxis(frame.strafe) || !validAxis(frame.armYaw) ||
                !validAxis(frame.armReach) || !validAxis(frame.armHeight) ||
                frame.gripper < -1 || frame.gripper > 1 ||
                (frame.controlFlags & ~EStopAsserted) != 0) {
                markMalformed();
                return;
            }
            acceptFrame(frame, nowMs);
            break;
        }
        case MessageType::Hello:
            if (payloadLength != 0) { markMalformed(); return; }
            requests_.flags |= RequestHello;
            break;
        case MessageType::Arm:
            if (payloadLength != 0) { markMalformed(); return; }
            requests_.flags |= RequestArm;
            break;
        case MessageType::Disarm:
            if (payloadLength != 0) { markMalformed(); return; }
            requests_.flags |= RequestDisarm;
            break;
        case MessageType::ClearEStop:
            if (payloadLength != 0) { markMalformed(); return; }
            requests_.flags |= RequestClearEStop;
            break;
        case MessageType::ClearFault:
            if (payloadLength != 0) { markMalformed(); return; }
            requests_.flags |= RequestClearFault;
            break;
        case MessageType::ParameterSet:
            if (payloadLength < 4 || parameter_.valid) { markMalformed(); return; }
            parameter_.valid = true;
            parameter_.group = static_cast<ParameterGroup>(payload[0]);
            parameter_.index = payload[1];
            parameter_.expectedRevision = readU16(payload + 2);
            parameter_.length = static_cast<uint8_t>(payloadLength - 4U);
            parameter_.sequence = sequence;
            memcpy(parameter_.data, payload + 4, parameter_.length);
            break;
        case MessageType::CalibrationCommand:
            if (payloadLength != 2 || calibration_.valid) { markMalformed(); return; }
            calibration_ = {true, payload[0], payload[1], sequence};
            break;
        case MessageType::DriveCalibrationCommand:
            if (payloadLength != 6 || driveCalibration_.valid) { markMalformed(); return; }
            driveCalibration_.valid = true;
            driveCalibration_.mode = payload[0];
            driveCalibration_.channel = payload[1];
            driveCalibration_.value = readI16(payload + 2);
            driveCalibration_.durationMs = readU16(payload + 4);
            driveCalibration_.sequence = sequence;
            break;
        case MessageType::ParameterSnapshotRequest:
            if (payloadLength != 0) { markMalformed(); return; }
            snapshotRequested_ = true;
            snapshotSequence_ = sequence;
            break;
        default:
            break;
    }
}

void CommunicationSubsystem::poll(Stream &stream, uint32_t nowMs) {
    uint8_t processed = 0;
    while (stream.available() > 0 && processed++ < 48) {
        const uint8_t value = static_cast<uint8_t>(stream.read());
        if (value == 0) {
            if (encodedLength_ > 0) finishFrame(nowMs);
        } else if (encodedLength_ < sizeof(encoded_)) {
            encoded_[encodedLength_++] = value;
        } else {
            encodedLength_ = 0;
            saturatingIncrement(rxOverflows_);
        }
    }
}

bool CommunicationSubsystem::sendFrame(
    Stream &stream, MessageType type, uint8_t sequence,
    const uint8_t *payload, uint8_t payloadLength
) {
    (void)stream;
    if (payloadLength > MaximumPayload) return false;
    if (transmitLength_ != 0) {
        saturatingIncrement(txDrops_);
        return false;
    }
    uint8_t raw[32];
    raw[0] = ProtocolVersion;
    raw[1] = static_cast<uint8_t>(type);
    raw[2] = sequence;
    raw[3] = payloadLength;
    if (payloadLength > 0 && payload != 0) memcpy(raw + 4, payload, payloadLength);
    raw[4 + payloadLength] = crc8(raw, static_cast<uint8_t>(4 + payloadLength));
    const uint8_t encodedLength = cobsEncode(
        raw, static_cast<uint8_t>(5 + payloadLength), transmit_, sizeof(transmit_) - 1
    );
    if (encodedLength == 0) return false;
    transmit_[encodedLength] = 0;
    transmitLength_ = static_cast<uint8_t>(encodedLength + 1U);
    transmitOffset_ = 0;
    return true;
}

bool CommunicationSubsystem::sendAck(
    Stream &stream, uint8_t sequence, MessageType acknowledged, uint16_t revision
) {
    const uint8_t payload[3] = {
        static_cast<uint8_t>(acknowledged), static_cast<uint8_t>(revision),
        static_cast<uint8_t>(revision >> 8)
    };
    return sendFrame(stream, MessageType::Ack, sequence, payload, sizeof(payload));
}

bool CommunicationSubsystem::sendNack(
    Stream &stream, uint8_t sequence, MessageType rejected, NackReason reason
) {
    const uint8_t payload[2] = {
        static_cast<uint8_t>(rejected), static_cast<uint8_t>(reason)
    };
    return sendFrame(stream, MessageType::Nack, sequence, payload, sizeof(payload));
}

const OperatorControlFrame &CommunicationSubsystem::latest() const {
    return latest_;
}

ControlRequests CommunicationSubsystem::takeRequests() {
    const ControlRequests result = requests_;
    requests_.flags = 0;
    return result;
}

bool CommunicationSubsystem::takeParameter(PendingParameter &parameter) {
    if (!parameter_.valid) return false;
    parameter = parameter_;
    parameter_.valid = false;
    return true;
}

bool CommunicationSubsystem::takeCalibration(PendingCalibration &calibration) {
    if (!calibration_.valid) return false;
    calibration = calibration_;
    calibration_.valid = false;
    return true;
}

bool CommunicationSubsystem::takeDriveCalibration(PendingDriveCalibration &calibration) {
    if (!driveCalibration_.valid) return false;
    calibration = driveCalibration_;
    driveCalibration_.valid = false;
    return true;
}

bool CommunicationSubsystem::takeSnapshotRequest(uint8_t &sequence) {
    if (!snapshotRequested_) return false;
    snapshotRequested_ = false;
    sequence = snapshotSequence_;
    return true;
}

uint16_t CommunicationSubsystem::rxOverflows() const { return rxOverflows_; }
uint16_t CommunicationSubsystem::malformedFrames() const { return malformedFrames_; }

void CommunicationSubsystem::pumpTransmit(Stream &stream, uint8_t byteBudget) {
    while (transmitLength_ != 0 && byteBudget-- > 0) {
        if (stream.write(transmit_[transmitOffset_]) != 1) return;
        ++transmitOffset_;
        if (transmitOffset_ >= transmitLength_) {
            transmitLength_ = 0;
            transmitOffset_ = 0;
        }
    }
}

bool CommunicationSubsystem::transmitIdle() const { return transmitLength_ == 0; }
uint16_t CommunicationSubsystem::txDrops() const { return txDrops_; }

} // namespace robot
