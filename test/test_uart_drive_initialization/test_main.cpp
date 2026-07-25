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

void initializeWithValidFeedback(
    UartEncoderDriveBackend &backend, HardwareSerial &serial,
    const RuntimeConfig &runtime
) {
    advanceToInitializationQuery(backend, serial, runtime);
    serial.queueReceive("$MOTOR_4CH_Encoder_20ms:0,0,0,0!");
    backend.pollReceive(449, runtime);
    TEST_ASSERT_TRUE(backend.health(449).feedbackHealthy);
}

void startNormalEncoderQuery(
    UartEncoderDriveBackend &backend, const RuntimeConfig &runtime,
    uint32_t nowMs
) {
    backend.onEncoderDeadline(nowMs, runtime);
    serviceAt(backend, runtime, nowMs);
    TEST_ASSERT_EQUAL_UINT8(1, backend.outstandingQuery());
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

void test_normal_query_accepts_a_legal_reply_after_15_ms() {
    HardwareSerial serial;
    UartEncoderDriveBackend backend(serial);
    const RuntimeConfig runtime = RuntimeConfig::defaults();
    initializeWithValidFeedback(backend, serial, runtime);

    startNormalEncoderQuery(backend, runtime, 450);
    backend.pollReceive(465, runtime);
    TEST_ASSERT_EQUAL_UINT8(1, backend.outstandingQuery());

    serial.queueReceive("$MOTOR_4CH_Encoder_20ms:1,2,3,4!");
    backend.pollReceive(479, runtime);
    TEST_ASSERT_EQUAL_UINT8(0, backend.outstandingQuery());
    TEST_ASSERT_EQUAL_UINT32(
        479, backend.feedback().incrementUpdatedAtMs
    );
    TEST_ASSERT_EQUAL_UINT16(0, backend.health(479).faults);
}

void test_complete_long_reply_at_30_ms_is_accepted_before_timeout() {
    HardwareSerial serial;
    UartEncoderDriveBackend backend(serial);
    RuntimeConfig runtime = RuntimeConfig::defaults();
    runtime.encoder.wheelDiameterMm = 34;
    runtime.encoder.countsPerRevolution = 65535;
    initializeWithValidFeedback(backend, serial, runtime);

    startNormalEncoderQuery(backend, runtime, 450);
    serial.queueReceive(
        "$MOTOR_4CH_Encoder_20ms:12000,-12000,12000,-12000!"
    );
    backend.pollReceive(
        450 + config::MotorBoardQueryTimeoutMs, runtime
    );

    TEST_ASSERT_EQUAL_UINT8(0, backend.outstandingQuery());
    TEST_ASSERT_EQUAL_UINT32(480, backend.feedback().incrementUpdatedAtMs);
    TEST_ASSERT_EQUAL_UINT8(0x0F, backend.feedback().encoderValidMask);
    TEST_ASSERT_EQUAL_UINT16(0, backend.health(480).faults);
}

void test_timeout_discards_partial_reply_and_pending_query() {
    HardwareSerial serial;
    UartEncoderDriveBackend backend(serial);
    const RuntimeConfig runtime = RuntimeConfig::defaults();
    initializeWithValidFeedback(backend, serial, runtime);

    startNormalEncoderQuery(backend, runtime, 450);
    backend.onEncoderDeadline(470, runtime);
    serial.queueReceive("$MOTOR_4CH_Encoder_20ms:1,2");
    backend.pollReceive(479, runtime);
    backend.pollReceive(480, runtime);
    serviceAt(backend, runtime, 480);
    serviceAt(backend, runtime, 481);

    TEST_ASSERT_EQUAL_UINT8(0, backend.outstandingQuery());
    TEST_ASSERT_EQUAL_UINT32(449, backend.feedback().incrementUpdatedAtMs);

    backend.onEncoderDeadline(490, runtime);
    serviceAt(backend, runtime, 490);
    TEST_ASSERT_EQUAL_UINT8(1, backend.outstandingQuery());
    serial.queueReceive(",3,4!");
    backend.pollReceive(491, runtime);

    TEST_ASSERT_EQUAL_UINT8(1, backend.outstandingQuery());
    TEST_ASSERT_EQUAL_UINT32(449, backend.feedback().incrementUpdatedAtMs);
}

void test_single_timeout_keeps_sample_and_success_resets_the_streak() {
    HardwareSerial serial;
    UartEncoderDriveBackend backend(serial);
    const RuntimeConfig runtime = RuntimeConfig::defaults();
    initializeWithValidFeedback(backend, serial, runtime);

    startNormalEncoderQuery(backend, runtime, 450);
    backend.pollReceive(
        450 + config::MotorBoardQueryTimeoutMs, runtime
    );

    const DriveFeedback &feedback = backend.feedback();
    TEST_ASSERT_EQUAL_UINT8(0, backend.outstandingQuery());
    TEST_ASSERT_EQUAL_UINT8(0x0F, feedback.encoderValidMask);
    TEST_ASSERT_EQUAL_UINT32(449, feedback.incrementUpdatedAtMs);
    TEST_ASSERT_TRUE(backend.health(480).feedbackHealthy);
    TEST_ASSERT_EQUAL_UINT16(0, backend.health(480).faults);

    startNormalEncoderQuery(backend, runtime, 481);
    serial.queueReceive("$MOTOR_4CH_Encoder_20ms:1,2,3,4!");
    backend.pollReceive(482, runtime);

    const uint32_t laterQueries[] = {483, 514};
    for (uint8_t index = 0; index < 2; ++index) {
        startNormalEncoderQuery(
            backend, runtime, laterQueries[index]
        );
        backend.pollReceive(
            laterQueries[index] + config::MotorBoardQueryTimeoutMs,
            runtime
        );
    }
    TEST_ASSERT_EQUAL_UINT16(0, backend.health(544).faults);
}

#if !ROBOT_DRIVER_TIMEOUT_UNSAFE
void test_three_consecutive_normal_query_timeouts_latch_stale_fault() {
    HardwareSerial serial;
    UartEncoderDriveBackend backend(serial);
    const RuntimeConfig runtime = RuntimeConfig::defaults();
    initializeWithValidFeedback(backend, serial, runtime);

    const uint32_t queryTimes[] = {450, 481, 512};
    for (uint8_t index = 0; index < 3; ++index) {
        startNormalEncoderQuery(
            backend, runtime, queryTimes[index]
        );
        backend.pollReceive(
            queryTimes[index] + config::MotorBoardQueryTimeoutMs,
            runtime
        );
        if (index < 2) {
            TEST_ASSERT_EQUAL_UINT16(
                0, backend.health(queryTimes[index] + 30).faults
            );
        }
    }

    const uint16_t faults = backend.health(542).faults;
    TEST_ASSERT_TRUE((faults & FaultEncoderStale) != 0);
    TEST_ASSERT_EQUAL_UINT16(0, faults & FaultEncoderMalformed);
    TEST_ASSERT_FALSE(backend.health(542).feedbackReady);
    TEST_ASSERT_FALSE(backend.health(542).feedbackHealthy);

    startNormalEncoderQuery(backend, runtime, 543);
    serial.queueReceive("$MOTOR_4CH_Encoder_20ms:1,2,3,4!");
    backend.pollReceive(544, runtime);
    TEST_ASSERT_TRUE(backend.health(544).feedbackHealthy);
    TEST_ASSERT_TRUE(
        (backend.health(544).faults & FaultEncoderStale) != 0
    );
    backend.clearFaults();
    TEST_ASSERT_EQUAL_UINT16(0, backend.health(544).faults);
}

void test_feedback_becomes_stale_at_the_configured_threshold() {
    HardwareSerial serial;
    UartEncoderDriveBackend backend(serial);
    const RuntimeConfig runtime = RuntimeConfig::defaults();
    initializeWithValidFeedback(backend, serial, runtime);

    backend.pollReceive(548, runtime);
    TEST_ASSERT_TRUE(backend.health(548).feedbackHealthy);
    TEST_ASSERT_EQUAL_UINT16(0, backend.health(548).faults);

    backend.pollReceive(549, runtime);
    TEST_ASSERT_FALSE(backend.health(549).feedbackHealthy);
    TEST_ASSERT_TRUE(
        (backend.health(549).faults & FaultEncoderStale) != 0
    );
}
#else
void test_unsafe_timeouts_keep_ready_and_raise_warnings() {
    HardwareSerial serial;
    UartEncoderDriveBackend backend(serial);
    const RuntimeConfig runtime = RuntimeConfig::defaults();
    initializeWithValidFeedback(backend, serial, runtime);

    const uint32_t queryTimes[] = {450, 481, 512};
    for (uint8_t index = 0; index < 3; ++index) {
        startNormalEncoderQuery(
            backend, runtime, queryTimes[index]
        );
        backend.pollReceive(
            queryTimes[index] + config::MotorBoardQueryTimeoutMs,
            runtime
        );
    }

    const DriveHealth timedOut = backend.health(542);
    TEST_ASSERT_EQUAL_UINT16(0, timedOut.faults);
    TEST_ASSERT_TRUE(timedOut.initialized);
    TEST_ASSERT_TRUE(timedOut.feedbackReady);
    TEST_ASSERT_TRUE(timedOut.feedbackHealthy);
    TEST_ASSERT_TRUE(
        (timedOut.warnings & WarningDriverTimeoutUnsafe) != 0
    );
    TEST_ASSERT_TRUE(
        (timedOut.warnings & WarningEncoderTimeoutIgnored) != 0
    );

    startNormalEncoderQuery(backend, runtime, 543);
    serial.queueReceive("$MOTOR_4CH_Encoder_20ms:1,2,3,4!");
    backend.pollReceive(544, runtime);
    const DriveHealth recovered = backend.health(544);
    TEST_ASSERT_EQUAL_UINT16(0, recovered.faults);
    TEST_ASSERT_TRUE(recovered.feedbackHealthy);
    TEST_ASSERT_TRUE(
        (recovered.warnings & WarningDriverTimeoutUnsafe) != 0
    );
    TEST_ASSERT_EQUAL_UINT16(
        0, recovered.warnings & WarningEncoderTimeoutIgnored
    );
}

void test_unsafe_sample_age_keeps_feedback_healthy_with_warning() {
    HardwareSerial serial;
    UartEncoderDriveBackend backend(serial);
    const RuntimeConfig runtime = RuntimeConfig::defaults();
    initializeWithValidFeedback(backend, serial, runtime);

    const DriveHealth stale = backend.health(549);
    TEST_ASSERT_EQUAL_UINT16(0, stale.faults);
    TEST_ASSERT_TRUE(stale.feedbackReady);
    TEST_ASSERT_TRUE(stale.feedbackHealthy);
    TEST_ASSERT_TRUE(
        (stale.warnings & WarningEncoderTimeoutIgnored) != 0
    );
}
#endif

#if ROBOT_CALIBRATION
void completeFreshIncrement(
    UartEncoderDriveBackend &backend, HardwareSerial &serial,
    const RuntimeConfig &runtime, uint32_t deadlineMs
) {
    backend.onEncoderDeadline(deadlineMs, runtime);
    serviceAt(backend, runtime, deadlineMs);
    TEST_ASSERT_EQUAL_UINT8(1, backend.outstandingQuery());
    serial.queueReceive("$MOTOR_4CH_Encoder_20ms:1,2,3,4!");
    backend.pollReceive(deadlineMs + 1, runtime);
    TEST_ASSERT_EQUAL_UINT8(0, backend.outstandingQuery());
}

void test_calibration_diagnostics_count_uart_frames_and_set_acks() {
    HardwareSerial serial;
    UartEncoderDriveBackend backend(serial);
    const RuntimeConfig runtime = RuntimeConfig::defaults();

    arduinoSetMillis(0);
    backend.begin(runtime);
    serviceAt(backend, runtime, 100);
    serial.queueReceive("$MOTOR_4CH_SET_OK:0!");
    backend.pollReceive(101, runtime);
    serviceAt(backend, runtime, 200);
    serial.queueReceive("$MOTOR_4CH_SET_ENCPDER_POLARITY_OK:0!");
    backend.pollReceive(201, runtime);
    serviceAt(backend, runtime, 300);
    serial.queueReceive("$MOTOR_4CH_Encoder_20ms:0,0,0,0!");
    backend.pollReceive(320, runtime);

    const DriveDiagnostics diagnostics = backend.diagnostics();
    TEST_ASSERT_EQUAL_UINT8(6, diagnostics.initializationStage);
    TEST_ASSERT_EQUAL_UINT8(89, diagnostics.receivedBytes);
    TEST_ASSERT_EQUAL_UINT8(3, diagnostics.completeFrames);
    TEST_ASSERT_EQUAL_UINT8(1, diagnostics.incrementFrames);
    TEST_ASSERT_EQUAL_UINT8(0x03, diagnostics.configurationAckMask);
}

void test_calibration_total_query_parses_all_four_i32_values() {
    HardwareSerial serial;
    UartEncoderDriveBackend backend(serial);
    const RuntimeConfig runtime = RuntimeConfig::defaults();
    advanceToInitializationQuery(backend, serial, runtime);
    serial.queueReceive("$MOTOR_4CH_Encoder_20ms:0,0,0,0!");
    backend.pollReceive(449, runtime);

    backend.onEncoderDeadline(450, runtime);
    serviceAt(backend, runtime, 450);
    TEST_ASSERT_EQUAL_UINT8(1, backend.outstandingQuery());
    backend.onEncoderDeadline(470, runtime);
    backend.onEncoderTotalDeadline(470);
    serial.queueReceive("$MOTOR_4CH_Encoder_20ms:1,2,3,4!");
    backend.pollReceive(475, runtime);
    serial.clearTransmit();
    serviceAt(backend, runtime, 475);

    TEST_ASSERT_EQUAL_UINT8(2, backend.outstandingQuery());
    TEST_ASSERT_NOT_EQUAL(
        std::string::npos,
        serial.transmit().find("$MOTOR_4CH_READ:encoder_total!")
    );

    serial.queueReceive(
        "$MOTOR_4CH_Encoder_Total:"
        "2147483647,-2147483648,123456,-654321!"
    );
    backend.pollReceive(476, runtime);
    backend.pollReceive(476, runtime);

    const DriveFeedback &feedback = backend.feedback();
    TEST_ASSERT_EQUAL_UINT8(0x0F, feedback.totalValidMask);
    TEST_ASSERT_EQUAL_INT32(2147483647L, feedback.total[0]);
    TEST_ASSERT_EQUAL_INT32(INT32_MIN, feedback.total[1]);
    TEST_ASSERT_EQUAL_INT32(123456L, feedback.total[2]);
    TEST_ASSERT_EQUAL_INT32(-654321L, feedback.total[3]);
}

void test_calibration_total_bad_reply_and_timeout_clear_validity() {
    HardwareSerial serial;
    UartEncoderDriveBackend backend(serial);
    const RuntimeConfig runtime = RuntimeConfig::defaults();
    advanceToInitializationQuery(backend, serial, runtime);
    serial.queueReceive("$MOTOR_4CH_Encoder_20ms:0,0,0,0!");
    backend.pollReceive(449, runtime);

    completeFreshIncrement(backend, serial, runtime, 450);
    backend.onEncoderTotalDeadline(452);
    serial.queueReceive("$MOTOR_4CH_Encoder_Total:1,2,3,4!");
    backend.pollReceive(453, runtime);
    TEST_ASSERT_EQUAL_UINT8(0x0F, backend.feedback().totalValidMask);

    completeFreshIncrement(backend, serial, runtime, 454);
    backend.onEncoderTotalDeadline(456);
    TEST_ASSERT_EQUAL_UINT8(0, backend.feedback().totalValidMask);
    serial.queueReceive("$MOTOR_4CH_Encoder_Total:1,2,bad,4!");
    backend.pollReceive(457, runtime);
    TEST_ASSERT_EQUAL_UINT8(0, backend.feedback().totalValidMask);

    completeFreshIncrement(backend, serial, runtime, 458);
    backend.onEncoderTotalDeadline(460);
    backend.pollReceive(
        460 + config::MotorBoardQueryTimeoutMs, runtime
    );
    TEST_ASSERT_EQUAL_UINT8(0, backend.feedback().totalValidMask);
    TEST_ASSERT_EQUAL_UINT8(0, backend.outstandingQuery());
}
#endif

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(
        test_initialization_waits_150_ms_then_enters_zero_output_retry
    );
    RUN_TEST(test_initialization_retries_at_10_second_boundary);
    RUN_TEST(test_vendor_prefix_alone_initializes_without_fake_feedback);
    RUN_TEST(test_one_parseable_vendor_reply_is_enough_for_feedback);
    RUN_TEST(test_normal_query_accepts_a_legal_reply_after_15_ms);
    RUN_TEST(
        test_complete_long_reply_at_30_ms_is_accepted_before_timeout
    );
    RUN_TEST(test_timeout_discards_partial_reply_and_pending_query);
    RUN_TEST(
        test_single_timeout_keeps_sample_and_success_resets_the_streak
    );
#if !ROBOT_DRIVER_TIMEOUT_UNSAFE
    RUN_TEST(
        test_three_consecutive_normal_query_timeouts_latch_stale_fault
    );
    RUN_TEST(
        test_feedback_becomes_stale_at_the_configured_threshold
    );
#else
    RUN_TEST(test_unsafe_timeouts_keep_ready_and_raise_warnings);
    RUN_TEST(
        test_unsafe_sample_age_keeps_feedback_healthy_with_warning
    );
#endif
#if ROBOT_CALIBRATION
    RUN_TEST(
        test_calibration_diagnostics_count_uart_frames_and_set_acks
    );
    RUN_TEST(test_calibration_total_query_parses_all_four_i32_values);
    RUN_TEST(
        test_calibration_total_bad_reply_and_timeout_clear_validity
    );
#endif
    return UNITY_END();
}
