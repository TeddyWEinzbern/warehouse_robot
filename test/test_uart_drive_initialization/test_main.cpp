#include <unity.h>

#include "app/BuildConfig.h"
#include "domain/RuntimeConfig.h"
#include "drivers/UartEncoderDriveBackend.h"

using namespace robot;

namespace {
void serviceAt(
    UartEncoderDriveBackend &backend, const RuntimeConfig &runtime,
    uint32_t nowMs
) {
    arduinoSetMillis(nowMs);
    backend.service(nowMs, runtime);
}

void advanceToInitializationQuery(
    UartEncoderDriveBackend &backend, HardwareSerial &serial,
    const RuntimeConfig &runtime
) {
    arduinoSetMillis(0);
    backend.begin(runtime);
    serviceAt(backend, runtime, 100);
    serviceAt(backend, runtime, 200);
    serviceAt(backend, runtime, 300);
    TEST_ASSERT_EQUAL_UINT8(1, backend.outstandingQuery());
    TEST_ASSERT_NOT_EQUAL(
        std::string::npos,
        serial.transmit().find("$MOTOR_4CH_READ:encoder_20ms!")
    );
}

void assertOnlyZeroFrames(const std::string &transmit) {
    const char zero[] = "$Car:0,0,0,0!";
    const size_t frameLength = sizeof(zero) - 1U;
    TEST_ASSERT_FALSE(transmit.empty());
    TEST_ASSERT_EQUAL_UINT32(0, transmit.size() % frameLength);
    for (size_t offset = 0; offset < transmit.size();
         offset += frameLength) {
        TEST_ASSERT_EQUAL_MEMORY(
            zero, transmit.data() + offset, frameLength
        );
    }
}
}

void test_initialization_waits_150_ms_then_enters_zero_output_retry() {
    HardwareSerial serial;
    UartEncoderDriveBackend backend(serial);
    const RuntimeConfig runtime = RuntimeConfig::defaults();
    advanceToInitializationQuery(backend, serial, runtime);

    backend.pollReceive(449, runtime);
    TEST_ASSERT_EQUAL_UINT8(1, backend.outstandingQuery());
    TEST_ASSERT_FALSE(backend.health(449).initialized);

    backend.pollReceive(450, runtime);
    TEST_ASSERT_EQUAL_UINT8(0, backend.outstandingQuery());
    TEST_ASSERT_FALSE(backend.health(450).initialized);
    TEST_ASSERT_EQUAL_UINT16(0, backend.health(450).faults);

    serial.clearTransmit();
    backend.setWheelTargets({200, -200, 200, -200});
    backend.onMotorDeadline(450, false, runtime);
    backend.onEncoderTotalDeadline(451);
    for (uint32_t nowMs = 450; nowMs < 10449; nowMs += 50)
        serviceAt(backend, runtime, nowMs);
    assertOnlyZeroFrames(serial.transmit());
}

void test_initialization_retries_at_10_second_boundary() {
    HardwareSerial serial;
    UartEncoderDriveBackend backend(serial);
    const RuntimeConfig runtime = RuntimeConfig::defaults();
    advanceToInitializationQuery(backend, serial, runtime);
    backend.pollReceive(450, runtime);

    serviceAt(backend, runtime, 10449);
    serial.clearTransmit();
    serviceAt(backend, runtime, 10450);
    TEST_ASSERT_TRUE(serial.transmit().empty());

    serial.clearTransmit();
    serviceAt(backend, runtime, 10549);
    assertOnlyZeroFrames(serial.transmit());

    serial.clearTransmit();
    serviceAt(backend, runtime, 10550);
    TEST_ASSERT_NOT_EQUAL(
        std::string::npos,
        serial.transmit().find("$MOTOR_4CH_SET:0!")
    );
}

void test_vendor_prefix_alone_initializes_without_fake_feedback() {
    HardwareSerial serial;
    UartEncoderDriveBackend backend(serial);
    const RuntimeConfig runtime = RuntimeConfig::defaults();
    advanceToInitializationQuery(backend, serial, runtime);

    serial.queueReceive(
        "$MOTOR_4CH_Encoder_20ms:vendor-format!"
    );
    backend.pollReceive(449, runtime);

    const DriveHealth health = backend.health(449);
    TEST_ASSERT_TRUE(health.initialized);
    TEST_ASSERT_FALSE(health.feedbackReady);
    TEST_ASSERT_FALSE(health.feedbackHealthy);
    TEST_ASSERT_EQUAL_UINT16(0, health.faults);
}

void test_one_parseable_vendor_reply_is_enough_for_feedback() {
    HardwareSerial serial;
    UartEncoderDriveBackend backend(serial);
    const RuntimeConfig runtime = RuntimeConfig::defaults();
    advanceToInitializationQuery(backend, serial, runtime);

    serial.queueReceive(
        "$MOTOR_4CH_Encoder_20ms:0,0,0,0!"
    );
    backend.pollReceive(449, runtime);

    const DriveHealth health = backend.health(449);
    TEST_ASSERT_TRUE(health.initialized);
    TEST_ASSERT_TRUE(health.feedbackReady);
    TEST_ASSERT_TRUE(health.feedbackHealthy);
    TEST_ASSERT_EQUAL_UINT8(0x0F, backend.feedback().encoderValidMask);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(
        test_initialization_waits_150_ms_then_enters_zero_output_retry
    );
    RUN_TEST(test_initialization_retries_at_10_second_boundary);
    RUN_TEST(test_vendor_prefix_alone_initializes_without_fake_feedback);
    RUN_TEST(test_one_parseable_vendor_reply_is_enough_for_feedback);
    return UNITY_END();
}
