#pragma once

#include <Arduino.h>

#include "domain/RuntimeConfig.h"
#include "domain/RobotTypes.h"

namespace robot {

enum class MessageType : uint8_t {
    Hello = 0x01,
    Control = 0x02,
    Arm = 0x03,
    Disarm = 0x04,
    ClearEStop = 0x05,
    ClearFault = 0x06,
    ParameterSet = 0x10,
    CalibrationCommand = 0x11,
    ParameterSnapshotRequest = 0x12,
    HelloTelemetry = 0x80,
    StatusTelemetry = 0x81,
    DriveCommandTelemetry = 0x82,
    DriveFeedbackTelemetry = 0x83,
    EncoderTotalsTelemetry = 0x84,
    SchedulerTelemetry = 0x85,
    SensorArmTelemetry = 0x86,
    OpenLoopPwmTelemetry = 0x87,
    ParameterSnapshot = 0x90,
    Ack = 0x91,
    Nack = 0x92,
};

enum class NackReason : uint8_t {
    Malformed = 1,
    Unsupported = 2,
    InvalidState = 3,
    RevisionMismatch = 4,
    ValidationFailed = 5,
};

struct PendingParameter {
    bool valid;
    ParameterGroup group;
    uint8_t index;
    uint16_t expectedRevision;
    uint8_t data[23];
    uint8_t length;
    uint8_t sequence;
};

struct PendingCalibration {
    bool valid;
    uint8_t joint;
    uint8_t degrees;
    uint8_t sequence;
};

class CommunicationSubsystem {
  public:
    CommunicationSubsystem();
    void poll(Stream &stream, uint32_t nowMs);
    const OperatorControlFrame &latest() const;
    ControlRequests takeRequests();
    bool takeParameter(PendingParameter &parameter);
    bool takeCalibration(PendingCalibration &calibration);
    bool takeSnapshotRequest(uint8_t &sequence);
    bool sendFrame(
        Stream &stream,
        MessageType type,
        uint8_t sequence,
        const uint8_t *payload,
        uint8_t payloadLength
    );
    bool sendAck(
        Stream &stream,
        uint8_t sequence,
        MessageType acknowledged,
        uint16_t revision
    );
    bool sendNack(
        Stream &stream,
        uint8_t sequence,
        MessageType rejected,
        NackReason reason
    );
    void pumpTransmit(Stream &stream, uint8_t byteBudget);
    bool transmitIdle() const;
    uint16_t txDrops() const;
    uint16_t rxOverflows() const;
    uint16_t malformedFrames() const;

  private:
    OperatorControlFrame latest_;
    ControlRequests requests_;
    PendingParameter parameter_;
    PendingCalibration calibration_;
    uint16_t previousButtons_;
    uint16_t rxOverflows_;
    uint16_t malformedFrames_;
    uint8_t snapshotSequence_;
    uint8_t encoded_[36];
    uint8_t encodedLength_;
    uint8_t transmit_[34];
    uint8_t transmitLength_;
    uint8_t transmitOffset_;
    uint16_t txDrops_;
    bool snapshotRequested_;
    void acceptFrame(OperatorControlFrame &frame, uint32_t nowMs);
    void finishFrame(uint32_t nowMs);
    void markMalformed();
    static uint8_t crc8(const uint8_t *data, uint8_t length);
    static uint8_t cobsDecode(
        const uint8_t *input, uint8_t length, uint8_t *output, uint8_t capacity
    );
    static uint8_t cobsEncode(
        const uint8_t *input, uint8_t length, uint8_t *output, uint8_t capacity
    );
};

} // namespace robot
