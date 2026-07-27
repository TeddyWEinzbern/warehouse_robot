#include <unity.h>

#include "subsystems/CommunicationSubsystem.h"

using namespace robot;

namespace {
void deliver(
    CommunicationSubsystem &receiver, MessageType type,
    uint8_t sequence, const uint8_t *payload, uint8_t payloadLength
) {
    CommunicationSubsystem sender;
    HardwareSerial encoded;
    TEST_ASSERT_TRUE(
        sender.sendFrame(type, sequence, payload, payloadLength)
    );
    sender.pumpTransmit(encoded, 255);

    HardwareSerial input;
    input.queueReceive(
        reinterpret_cast<const uint8_t *>(encoded.transmit().data()),
        encoded.transmit().size()
    );
    receiver.poll(input, 123);
}
}

void test_calibration_commands_match_the_firmware_profile() {
    CommunicationSubsystem communication;

    const uint8_t armMove[] = {2, 90};
    deliver(
        communication, MessageType::CalibrationArmMove,
        11, armMove, sizeof(armMove)
    );

#if ROBOT_CALIBRATION
    PendingArmMove pendingArm = {};
    TEST_ASSERT_TRUE(communication.takeArmMove(pendingArm));
    TEST_ASSERT_EQUAL_UINT8(2, pendingArm.joint);
    TEST_ASSERT_EQUAL_UINT8(90, pendingArm.degrees);
    TEST_ASSERT_EQUAL_UINT8(11, pendingArm.sequence);

    const uint8_t reference[] = {1, 20, 160, 0xFD, 1};
    deliver(
        communication, MessageType::CalibrationSetJointReference,
        12, reference, sizeof(reference)
    );
    PendingJointReference pendingReference = {};
    TEST_ASSERT_TRUE(
        communication.takeJointReference(pendingReference)
    );
    TEST_ASSERT_EQUAL_UINT8(1, pendingReference.joint);
    TEST_ASSERT_EQUAL_INT8(-3, pendingReference.centerOffsetDegrees);

    const uint8_t spin[] = {1, 2, 0xD4, 0xFE, 0xF4, 0x01};
    deliver(
        communication, MessageType::CalibrationDriveSpin,
        13, spin, sizeof(spin)
    );
    PendingDriveCalibration pendingSpin = {};
    TEST_ASSERT_TRUE(
        communication.takeDriveCalibration(pendingSpin)
    );
    TEST_ASSERT_EQUAL_INT16(-300, pendingSpin.value);
    TEST_ASSERT_EQUAL_UINT16(500, pendingSpin.durationMs);

    const uint8_t page[] = {1};
    deliver(
        communication, MessageType::CalibrationReadDrive,
        14, page, sizeof(page)
    );
    PendingCalibrationRead pendingRead = {};
    TEST_ASSERT_TRUE(
        communication.takeCalibrationRead(pendingRead)
    );
    TEST_ASSERT_EQUAL_UINT8(1, pendingRead.page);
    TEST_ASSERT_EQUAL_UINT8(14, pendingRead.sequence);

    HardwareSerial reply;
    TEST_ASSERT_TRUE(
        communication.sendAck(
            15, MessageType::CalibrationDriveSpin
        )
    );
    communication.pumpTransmit(reply, 255);
    TEST_ASSERT_FALSE(reply.transmit().empty());
#else
    const uint8_t reference[] = {1, 20, 160, 0xFD, 1};
    const uint8_t spin[] = {1, 2, 0xD4, 0xFE, 0xF4, 0x01};
    const uint8_t page[] = {1};
    deliver(
        communication, MessageType::CalibrationSetJointReference,
        12, reference, sizeof(reference)
    );
    deliver(
        communication, MessageType::CalibrationDriveSpin,
        13, spin, sizeof(spin)
    );
    deliver(
        communication, MessageType::CalibrationReadDrive,
        14, page, sizeof(page)
    );
    deliver(
        communication, MessageType::CalibrationReadSensor,
        15, 0, 0
    );
    deliver(
        communication, MessageType::CalibrationReadArm,
        16, 0, 0
    );
    deliver(
        communication, MessageType::CalibrationReadSystem,
        17, 0, 0
    );
    TEST_ASSERT_EQUAL_UINT8(
        0, communication.takeRequests().flags
    );
    TEST_ASSERT_TRUE(communication.transmitIdle());
#endif

    deliver(
        communication, MessageType::Hello,
        21, 0, 0
    );
    TEST_ASSERT_EQUAL_UINT8(
        RequestHello, communication.takeRequests().flags
    );
}

#if ROBOT_CALIBRATION
void test_disarm_discards_pending_calibration_actuator_commands() {
    CommunicationSubsystem communication;
    const uint8_t armMove[] = {2, 90};
    const uint8_t reference[] = {1, 20, 160, 0xFD, 1};
    const uint8_t spin[] = {1, 2, 0xD4, 0xFE, 0xF4, 0x01};
    deliver(
        communication, MessageType::CalibrationArmMove,
        11, armMove, sizeof(armMove)
    );
    deliver(
        communication, MessageType::CalibrationSetJointReference,
        12, reference, sizeof(reference)
    );
    deliver(
        communication, MessageType::CalibrationDriveSpin,
        13, spin, sizeof(spin)
    );
    deliver(communication, MessageType::Disarm, 14, 0, 0);

    TEST_ASSERT_EQUAL_UINT8(
        RequestDisarm, communication.takeRequests().flags
    );
    PendingArmMove pendingArm = {};
    PendingJointReference pendingReference = {};
    PendingDriveCalibration pendingSpin = {};
    TEST_ASSERT_FALSE(communication.takeArmMove(pendingArm));
    TEST_ASSERT_FALSE(
        communication.takeJointReference(pendingReference)
    );
    TEST_ASSERT_FALSE(
        communication.takeDriveCalibration(pendingSpin)
    );
}
#endif

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_calibration_commands_match_the_firmware_profile);
#if ROBOT_CALIBRATION
    RUN_TEST(test_disarm_discards_pending_calibration_actuator_commands);
#endif
    return UNITY_END();
}
