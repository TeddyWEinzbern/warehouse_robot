#pragma once

#include <Arduino.h>

#include "app/BuildConfig.h"
#include "domain/RobotTypes.h"

namespace robot {

constexpr uint8_t ProtocolVersion = 3;

enum class MessageType : uint8_t {
    Hello = 0x01,
    Arm = 0x03,
    Disarm = 0x04,
    ClearEStop = 0x05,
    ClearFault = 0x06,
    CalibrationArmMove = 0x10,
    CalibrationSetJointReference = 0x11,
    CalibrationDriveSpin = 0x12,
    CalibrationReadDrive = 0x13,
    CalibrationReadSensor = 0x14,
    CalibrationReadArm = 0x15,
    CalibrationReadSystem = 0x16,
    HelloResponse = 0x80,
    CriticalStatus = 0x81,
    CalibrationReport = 0x90,
    Ack = 0x91,
    Nack = 0x92,
};

enum class NackReason : uint8_t {
    Malformed = 1,
    Unsupported = 2,
    InvalidState = 3,
    ValidationFailed = 4,
};

enum class CalibrationReportKind : uint8_t {
    Arm = 1,
    DriveCounts = 2,
    DriveSpeed = 3,
    Sensor = 4,
    System = 5,
};

struct PendingArmMove {
    bool valid;
    uint8_t joint;
    uint8_t degrees;
    uint8_t sequence;
};

struct PendingJointReference {
    bool valid;
    uint8_t joint;
    uint8_t lowerDegrees;
    uint8_t upperDegrees;
    int8_t centerOffsetDegrees;
    int8_t direction;
    uint8_t sequence;
};

struct PendingDriveCalibration {
    bool valid;
    uint8_t mode;
    uint8_t channel;
    int16_t value;
    uint16_t durationMs;
    uint8_t sequence;
};

struct PendingCalibrationRead {
    bool valid;
    MessageType type;
    uint8_t page;
    uint8_t sequence;
};

class CommunicationSubsystem {
  public:
    CommunicationSubsystem();
    void poll(Stream &stream, uint32_t nowMs);
    const OperatorControlFrame &latest() const;
    ControlRequests takeRequests();
    bool takeArmMove(PendingArmMove &command);
    bool takeJointReference(PendingJointReference &command);
    bool takeDriveCalibration(PendingDriveCalibration &command);
    bool takeCalibrationRead(PendingCalibrationRead &request);
    bool sendFrame(
        MessageType type,
        uint8_t sequence,
        const uint8_t *payload,
        uint8_t payloadLength
    );
    bool sendAck(uint8_t sequence, MessageType acknowledged);
    bool sendNack(
        uint8_t sequence,
        MessageType rejected,
        NackReason reason
    );
    void pumpTransmit(Stream &stream, uint8_t byteBudget);
    bool transmitIdle() const;
    bool receiveIdle() const;

  private:
    static constexpr uint8_t MaximumPayload =
        ROBOT_CALIBRATION ? 27 : 8;
    static constexpr uint8_t MaximumRaw = MaximumPayload + 5;
    static constexpr uint8_t MaximumEncoded = MaximumRaw + 2;

    OperatorControlFrame latest_;
    ControlRequests requests_;
    PendingArmMove armMove_;
    PendingJointReference jointReference_;
    PendingDriveCalibration driveCalibration_;
    PendingCalibrationRead calibrationRead_;
    uint8_t previousButtons_;
    uint8_t encoded_[MaximumEncoded];
    uint8_t raw_[MaximumRaw];
    uint8_t transmit_[MaximumEncoded];
    uint8_t encodedLength_;
    uint8_t transmitLength_;
    uint8_t transmitOffset_;
    bool discarding_;

    void acceptControl(const uint8_t *raw, uint32_t nowMs);
    void finishFrame(uint32_t nowMs);
    void acceptGeneric(uint8_t length);
    static int16_t expandAxis(uint8_t value);
    static int16_t expandTernary(uint8_t code);
    static uint8_t crc8(const uint8_t *data, uint8_t length);
    static uint8_t cobsDecode(
        const uint8_t *input,
        uint8_t length,
        uint8_t *output,
        uint8_t capacity
    );
    static uint8_t cobsEncode(
        const uint8_t *input,
        uint8_t length,
        uint8_t *output,
        uint8_t capacity
    );
};

} // namespace robot
