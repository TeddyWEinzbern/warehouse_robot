#include <unity.h>

#include "app/BuildConfig.h"
#include "core/MotorBoardProtocol.h"
#include "core/Scheduler.h"
#include "domain/ArmKinematics.h"
#include "domain/MecanumKinematics.h"
#include "domain/RuntimeConfig.h"
#include "domain/SafetyGates.h"
#include "drivers/DriveBackend.h"
#include "subsystems/ChassisSubsystem.h"
#include "subsystems/CommunicationSubsystem.h"
#include "subsystems/SafetySupervisor.h"

using namespace robot;

namespace {
class MockDriveBackend : public DriveBackend {
  public:
    WheelTargets stored = {0, 0, 0, 0};
    DriveFeedback driveFeedback = {};
    uint16_t stops = 0;
    void begin(const RuntimeConfig &) {}
    void pollReceive(uint32_t, const RuntimeConfig &) {}
    void service(uint32_t, const RuntimeConfig &) {}
    void setWheelTargets(const WheelTargets &targets) { stored = targets; }
    void onMotorDeadline(uint32_t, bool, const RuntimeConfig &) {}
    void onEncoderDeadline(uint32_t, const RuntimeConfig &) {}
#if ROBOT_CALIBRATION
    void onEncoderTotalDeadline(uint32_t) {}
#endif
    void stop(uint32_t) { ++stops; stored = {0, 0, 0, 0}; }
    const DriveFeedback &feedback() const { return driveFeedback; }
    DriveHealth health(uint32_t) const { return {0, 0, true, true, true}; }
    void clearFaults() {}
    uint8_t outstandingQuery() const { return 0; }
};

OperatorControlFrame neutralFrame(uint32_t receivedAtMs) {
    OperatorControlFrame frame = {};
    frame.valid = true;
    frame.receivedAtMs = receivedAtMs;
    return frame;
}

void qualifyNeutral(
    SafetySupervisor &safety,
    const DriveHealth &drive,
    bool platformInitialized = true,
    bool profileCanArm = true
) {
    OperatorControlFrame frame = neutralFrame(0);
    safety.update(
        frame, {0}, drive, platformInitialized, profileCanArm, 0
    );
    frame.receivedAtMs = config::NeutralQualificationMs;
    safety.update(
        frame, {0}, drive, platformInitialized, profileCanArm,
        config::NeutralQualificationMs
    );
}
} // namespace

void test_mecanum_forward_and_rotation_geometry() {
    const WheelTargets forward = MecanumKinematics::mix({400, 0, 0}, 160, 170, 1000);
    TEST_ASSERT_EQUAL_INT16(400, forward.frontLeft);
    TEST_ASSERT_EQUAL_INT16(400, forward.frontRight);
    TEST_ASSERT_EQUAL_INT16(400, forward.rearLeft);
    TEST_ASSERT_EQUAL_INT16(400, forward.rearRight);
    const WheelTargets rotation = MecanumKinematics::mix({0, 0, 1000}, 160, 170, 1000);
    TEST_ASSERT_EQUAL_INT16(165, rotation.frontLeft);
    TEST_ASSERT_EQUAL_INT16(-165, rotation.frontRight);
    TEST_ASSERT_EQUAL_INT16(165, rotation.rearLeft);
    TEST_ASSERT_EQUAL_INT16(-165, rotation.rearRight);
}

void test_mecanum_proportional_scaling_preserves_ratios() {
    const WheelTargets wheels = MecanumKinematics::mix({1000, 500, 0}, 160, 170, 600);
    TEST_ASSERT_EQUAL_INT16(600, wheels.frontLeft);
    TEST_ASSERT_EQUAL_INT16(200, wheels.frontRight);
    TEST_ASSERT_EQUAL_INT16(200, wheels.rearLeft);
    TEST_ASSERT_EQUAL_INT16(600, wheels.rearRight);
}

void test_arm_reachable_and_unreachable() {
    TEST_ASSERT_TRUE(ArmKinematics::solvePlanar(100, 80, 110, 110).reachable);
    TEST_ASSERT_FALSE(ArmKinematics::solvePlanar(300, 0, 110, 110).reachable);
}

void test_scheduler_uses_accumulated_deadlines_without_burst() {
    PeriodicTask task;
    task.start(0, 20000, 0);
    TEST_ASSERT_TRUE(task.due(0));
    TEST_ASSERT_FALSE(task.due(10000));
    TEST_ASSERT_TRUE(task.due(55000));
    TEST_ASSERT_EQUAL_UINT16(1, task.stats().missed);
    TEST_ASSERT_EQUAL_UINT32(35000, task.lastLatenessUs());
    TEST_ASSERT_FALSE(task.due(55000));
    TEST_ASSERT_EQUAL_UINT32(5000, task.untilDeadlineUs(55000));
    TEST_ASSERT_TRUE(task.due(61000));
    TEST_ASSERT_EQUAL_UINT32(1000, task.lastLatenessUs());
}

void test_lightweight_deadline_skips_missed_periods() {
    PeriodicDeadline task;
    task.start(1000, 20000, 5000);
    TEST_ASSERT_FALSE(task.due(5999));
    TEST_ASSERT_TRUE(task.due(6000));
    TEST_ASSERT_TRUE(task.due(51000));
    TEST_ASSERT_EQUAL_UINT32(15000, task.untilDeadlineUs(51000));
    TEST_ASSERT_FALSE(task.due(51000));
}

void test_chassis_ramp_and_controlled_zero_crossing() {
    MockDriveBackend backend;
    ChassisSubsystem chassis(backend);
    RuntimeConfig runtime = RuntimeConfig::defaults();
    chassis.setDesired({1000, 0, 0, 1000, IntentSource::Operator}, runtime);
    chassis.trajectoryTick(10000, 10000, runtime);
    TEST_ASSERT_EQUAL_INT16(6, chassis.rampedVelocity().longitudinalMmS);
    for (uint32_t now = 20000; now <= 200000; now += 10000)
        chassis.trajectoryTick(now, 10000, runtime);
    TEST_ASSERT_EQUAL_INT16(120, chassis.rampedVelocity().longitudinalMmS);

    chassis.setDesired({-1000, 0, 0, 1000, IntentSource::Operator}, runtime);
    uint32_t now = 210000;
    while (chassis.rampedVelocity().longitudinalMmS > 0) {
        chassis.trajectoryTick(now, 10000, runtime);
        now += 10000;
    }
    TEST_ASSERT_EQUAL_INT16(0, chassis.rampedVelocity().longitudinalMmS);
    TEST_ASSERT_BITS_HIGH(0x01, chassis.zeroCrossingMask(now));
    chassis.trajectoryTick(now + 20000, 20000, runtime);
    TEST_ASSERT_EQUAL_INT16(0, chassis.rampedVelocity().longitudinalMmS);
    TEST_ASSERT_BITS_HIGH(0x01, chassis.zeroCrossingMask(now + 20000));
    chassis.trajectoryTick(now + 50000, 30000, runtime);
    TEST_ASSERT_TRUE(chassis.rampedVelocity().longitudinalMmS < 0);
    TEST_ASSERT_BITS_LOW(0x01, chassis.zeroCrossingMask(now + 50000));
}

void test_force_zero_bypasses_ramp() {
    MockDriveBackend backend;
    ChassisSubsystem chassis(backend);
    RuntimeConfig runtime = RuntimeConfig::defaults();
    chassis.setDesired({1000, 0, 0, 1000, IntentSource::Operator}, runtime);
    chassis.trajectoryTick(50000, 50000, runtime);
    TEST_ASSERT_TRUE(chassis.rampedVelocity().longitudinalMmS > 0);
    chassis.forceZero(50);
    TEST_ASSERT_EQUAL_INT16(0, chassis.rampedVelocity().longitudinalMmS);
    TEST_ASSERT_EQUAL_UINT16(1, backend.stops);
}

void test_robot_state_and_protocol_values_are_stable() {
    TEST_ASSERT_EQUAL_UINT8(
        0, static_cast<uint8_t>(RobotState::Disarmed)
    );
    TEST_ASSERT_EQUAL_UINT8(
        1, static_cast<uint8_t>(RobotState::Armed)
    );
    TEST_ASSERT_EQUAL_UINT8(
        2, static_cast<uint8_t>(RobotState::Fault)
    );
    TEST_ASSERT_EQUAL_UINT8(4, ProtocolVersion);
    TEST_ASSERT_EQUAL_UINT8(
        0x04, static_cast<uint8_t>(MessageType::Disarm)
    );
    TEST_ASSERT_EQUAL_UINT8(
        0x05, static_cast<uint8_t>(MessageType::Reserved)
    );
    TEST_ASSERT_EQUAL_UINT8(
        0x06, static_cast<uint8_t>(MessageType::ClearFault)
    );
    TEST_ASSERT_EQUAL_UINT8(0x01, CriticalStatusLinkAlive);
    TEST_ASSERT_EQUAL_UINT8(0x02, CriticalStatusReadyToArm);
}

void test_ready_to_arm_requires_every_authoritative_input() {
    const DriveHealth healthy = {0, 0, true, true, true};

    SafetySupervisor initial;
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(RobotState::Disarmed),
        static_cast<uint8_t>(initial.state())
    );
    TEST_ASSERT_FALSE(initial.readyToArm());

    SafetySupervisor ready;
    qualifyNeutral(ready, healthy);
    TEST_ASSERT_TRUE(ready.readyToArm());

    SafetySupervisor noPlatform;
    qualifyNeutral(noPlatform, healthy, false, true);
    TEST_ASSERT_FALSE(noPlatform.readyToArm());

    SafetySupervisor noProfile;
    qualifyNeutral(noProfile, healthy, true, false);
    TEST_ASSERT_FALSE(noProfile.readyToArm());

    SafetySupervisor notInitialized;
    qualifyNeutral(
        notInitialized, {0, 0, false, true, true}
    );
    TEST_ASSERT_FALSE(notInitialized.readyToArm());

    SafetySupervisor noFeedback;
    qualifyNeutral(noFeedback, {0, 0, true, false, true});
    TEST_ASSERT_FALSE(noFeedback.readyToArm());

    SafetySupervisor unhealthy;
    qualifyNeutral(unhealthy, {0, 0, true, true, false});
    TEST_ASSERT_FALSE(unhealthy.readyToArm());

    SafetySupervisor faulted;
    qualifyNeutral(
        faulted, {FaultDriveStall, 0, true, true, true}
    );
    TEST_ASSERT_FALSE(faulted.readyToArm());

    SafetySupervisor staleLink;
    OperatorControlFrame stale = neutralFrame(0);
    staleLink.update(stale, {0}, healthy, true, true, 0);
    staleLink.update(
        stale, {0}, healthy, true, true,
        config::NeutralQualificationMs
    );
    TEST_ASSERT_FALSE(staleLink.linkAlive());
    TEST_ASSERT_FALSE(staleLink.readyToArm());

    SafetySupervisor notQualified;
    OperatorControlFrame fresh = neutralFrame(0);
    notQualified.update(fresh, {0}, healthy, true, true, 0);
    fresh.receivedAtMs = config::NeutralQualificationMs - 1U;
    notQualified.update(
        fresh, {0}, healthy, true, true,
        config::NeutralQualificationMs - 1U
    );
    TEST_ASSERT_FALSE(notQualified.readyToArm());
}

void test_calibration_actions_require_platform_and_drive_initialization() {
    const DriveHealth initialized = {0, 0, true, true, true};
    const DriveHealth notInitialized = {0, 0, false, true, true};
    TEST_ASSERT_TRUE(calibrationActionsReady(true, initialized));
    TEST_ASSERT_FALSE(calibrationActionsReady(false, initialized));
    TEST_ASSERT_FALSE(calibrationActionsReady(true, notInitialized));
}

void test_arm_requires_ready_then_link_loss_disarms_immediately() {
    SafetySupervisor safety;
    OperatorControlFrame frame = neutralFrame(0);
    DriveHealth healthy = {0, 0, true, true, true};
    safety.update(frame, {0}, healthy, true, true, 0);
    safety.update(frame, {RequestArm}, healthy, true, true, 0);
    TEST_ASSERT_FALSE(safety.armed());
    frame.receivedAtMs = config::NeutralQualificationMs;
    safety.update(
        frame, {RequestArm}, healthy, true, true,
        config::NeutralQualificationMs
    );
    TEST_ASSERT_TRUE(safety.armed());
    TEST_ASSERT_FALSE(safety.readyToArm());
    frame.receivedAtMs = config::NeutralQualificationMs;
    safety.update(
        frame, {0}, healthy, true, true,
        config::NeutralQualificationMs + config::CommandTimeoutMs + 1U
    );
    TEST_ASSERT_FALSE(safety.armed());
    TEST_ASSERT_TRUE(safety.takeImmediateStop());
}

void test_simultaneous_disarm_and_arm_stays_disarmed() {
    SafetySupervisor safety;
    OperatorControlFrame frame = neutralFrame(0);
    DriveHealth healthy = {0, 0, true, true, true};
    qualifyNeutral(safety, healthy);
    TEST_ASSERT_TRUE(safety.takeImmediateStop());
    frame.receivedAtMs = config::NeutralQualificationMs;
    safety.update(
        frame, {RequestArm}, healthy, true, true,
        config::NeutralQualificationMs
    );
    TEST_ASSERT_TRUE(safety.armed());
    TEST_ASSERT_FALSE(safety.takeImmediateStop());
    safety.update(
        frame,
        {static_cast<uint8_t>(RequestDisarm | RequestArm)},
        healthy, true, true, config::NeutralQualificationMs
    );
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(RobotState::Disarmed),
        static_cast<uint8_t>(safety.state())
    );
    TEST_ASSERT_TRUE(safety.takeImmediateStop());
}

void test_fault_requires_explicit_clear_and_never_auto_arms() {
    SafetySupervisor safety;
    OperatorControlFrame frame = neutralFrame(0);
    DriveHealth healthy = {0, 0, true, true, true};
    qualifyNeutral(safety, healthy);
    safety.latchFault(FaultSchedulerOverrun);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(RobotState::Fault),
        static_cast<uint8_t>(safety.state())
    );

    frame.receivedAtMs = config::NeutralQualificationMs;
    safety.update(
        frame,
        {static_cast<uint8_t>(RequestDisarm | RequestClearFault |
                              RequestArm)},
        healthy, true, true, config::NeutralQualificationMs
    );
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(RobotState::Fault),
        static_cast<uint8_t>(safety.state())
    );
    TEST_ASSERT_EQUAL_HEX16(
        FaultSchedulerOverrun, safety.faults()
    );

    safety.update(
        frame,
        {static_cast<uint8_t>(RequestClearFault | RequestArm)},
        healthy, true, true, config::NeutralQualificationMs
    );
    TEST_ASSERT_TRUE(safety.takeClearFaultAccepted());
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(RobotState::Disarmed),
        static_cast<uint8_t>(safety.state())
    );
    TEST_ASSERT_FALSE(safety.armed());
    TEST_ASSERT_TRUE(safety.readyToArm());
}

void test_urgent_frame_is_disarm_and_reserved_generic_is_ignored() {
    CommunicationSubsystem communication;
    HardwareSerial urgent;
    const uint8_t urgentDisarm[4] = {0x03, 0x80, 0x89, 0x00};
    urgent.queueReceive(urgentDisarm, sizeof(urgentDisarm));
    communication.poll(urgent, 10);
    TEST_ASSERT_EQUAL_UINT8(
        RequestDisarm, communication.takeRequests().flags
    );

    CommunicationSubsystem sender;
    HardwareSerial encoded;
    TEST_ASSERT_TRUE(
        sender.sendFrame(MessageType::Reserved, 1, 0, 0)
    );
    sender.pumpTransmit(encoded, 255);
    HardwareSerial input;
    input.queueReceive(
        reinterpret_cast<const uint8_t *>(
            encoded.transmit().data()
        ),
        encoded.transmit().size()
    );
    communication.poll(input, 20);
    TEST_ASSERT_EQUAL_UINT8(0, communication.takeRequests().flags);

    HardwareSerial genericEncoded;
    TEST_ASSERT_TRUE(
        sender.sendFrame(MessageType::Disarm, 2, 0, 0)
    );
    sender.pumpTransmit(genericEncoded, 255);
    HardwareSerial genericInput;
    genericInput.queueReceive(
        reinterpret_cast<const uint8_t *>(
            genericEncoded.transmit().data()
        ),
        genericEncoded.transmit().size()
    );
    communication.poll(genericInput, 30);
    TEST_ASSERT_EQUAL_UINT8(
        RequestDisarm, communication.takeRequests().flags
    );
}

void test_held_button_cannot_neutral_qualify_for_arm() {
    SafetySupervisor safety;
    OperatorControlFrame frame = neutralFrame(0);
    DriveHealth healthy = {0, 0, true, true, true};
    safety.update(frame, {0}, healthy, true, true, 0);
    frame.buttons = PresetLeft;
    frame.receivedAtMs = config::NeutralQualificationMs;
    safety.update(
        frame, {RequestArm}, healthy, true, true,
        config::NeutralQualificationMs
    );
    TEST_ASSERT_FALSE(safety.armed());
    frame.buttons = 0;
    frame.receivedAtMs = config::NeutralQualificationMs + 100U;
    safety.update(
        frame, {0}, healthy, true, true,
        config::NeutralQualificationMs + 100U
    );
    frame.receivedAtMs =
        config::NeutralQualificationMs * 2U + 100U;
    safety.update(
        frame, {RequestArm}, healthy, true, true,
        config::NeutralQualificationMs * 2U + 100U
    );
    TEST_ASSERT_TRUE(safety.armed());
}

void test_calibration_servo_reference_validation_is_atomic() {
    RuntimeConfig runtime = RuntimeConfig::defaults();
    const ServoCalibration original = runtime.servos[0];
    TEST_ASSERT_FALSE(runtime.setCalibrationServoReference(
        0, 170, 180, 0, 1
    ));
    TEST_ASSERT_EQUAL_UINT8(
        original.lowerDegrees, runtime.servos[0].lowerDegrees
    );
    TEST_ASSERT_EQUAL_INT8(
        original.direction, runtime.servos[0].direction
    );
    TEST_ASSERT_TRUE(runtime.setCalibrationServoReference(
        0, 10, 170, 5, -1
    ));
    TEST_ASSERT_EQUAL_UINT8(10, runtime.servos[0].lowerDegrees);
    TEST_ASSERT_EQUAL_UINT8(170, runtime.servos[0].upperDegrees);
    TEST_ASSERT_EQUAL_INT8(5, runtime.servos[0].centerOffsetDegrees);
    TEST_ASSERT_EQUAL_INT8(-1, runtime.servos[0].direction);
}

void test_runtime_defaults_keep_calibrated_motor_mapping() {
    const RuntimeConfig runtime = RuntimeConfig::defaults();
    const int8_t commandMap[4] = {0, 2, 1, 3};
    const int8_t commandSigns[4] = {1, -1, -1, 1};
    const int8_t encoderMap[4] = {0, 1, 2, 3};
    const int8_t encoderSigns[4] = {-1, 1, -1, 1};
    for (uint8_t index = 0; index < 4; ++index) {
        TEST_ASSERT_EQUAL_INT8(
            commandMap[index], runtime.encoder.commandMap[index]
        );
        TEST_ASSERT_EQUAL_INT8(
            commandSigns[index], runtime.encoder.commandSigns[index]
        );
        TEST_ASSERT_EQUAL_INT8(
            encoderMap[index], runtime.encoder.channelMap[index]
        );
        TEST_ASSERT_EQUAL_INT8(
            encoderSigns[index], runtime.encoder.signs[index]
        );
    }
}

void test_motor_parser_resynchronizes_and_bounds_frames() {
    MotorBoardFrameParser parser;
    const char noise[] = "cmdOkgarbage";
    for (uint8_t i = 0; noise[i] != '\0'; ++i)
        TEST_ASSERT_EQUAL_UINT8(
            static_cast<uint8_t>(MotorBoardFeedResult::None),
            static_cast<uint8_t>(parser.feed(noise[i]))
        );
    const char fragmented[] = "$old-partial$MOTOR_4CH_Encoder_20ms:1,-2,3,-4!";
    MotorBoardFeedResult result = MotorBoardFeedResult::None;
    for (uint8_t i = 0; fragmented[i] != '\0'; ++i) result = parser.feed(fragmented[i]);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(MotorBoardFeedResult::Complete),
        static_cast<uint8_t>(result)
    );
    TEST_ASSERT_EQUAL_STRING("$MOTOR_4CH_Encoder_20ms:1,-2,3,-4!", parser.frame());

    const char longest[] =
        "$MOTOR_4CH_Encoder_Total:"
        "-2147483648,-2147483648,-2147483648,-2147483648!";
    static_assert(sizeof(longest) <= 80, "motor parser no longer fits totals");
    for (uint8_t i = 0; longest[i] != '\0'; ++i)
        result = parser.feed(longest[i]);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(MotorBoardFeedResult::Complete),
        static_cast<uint8_t>(result)
    );
    TEST_ASSERT_EQUAL_STRING(longest, parser.frame());

    parser.feed('$');
    for (uint8_t i = 0; i < 100; ++i) parser.feed('9');
    TEST_ASSERT_EQUAL_UINT16(1, parser.overflows());
    const char recovered[] = "$ok!";
    for (uint8_t i = 0; recovered[i] != '\0'; ++i) result = parser.feed(recovered[i]);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(MotorBoardFeedResult::Complete),
        static_cast<uint8_t>(result)
    );
    TEST_ASSERT_EQUAL_STRING("$ok!", parser.frame());
}

void test_motor_numeric_parser_rejects_malformed_and_out_of_range_values() {
    int32_t values[4] = {};
    TEST_ASSERT_TRUE(MotorBoardFrameParser::parseFour(
        "-2147483648,2147483647,0,-1!", values, INT32_MIN, INT32_MAX
    ));
    TEST_ASSERT_EQUAL_INT32(INT32_MIN, values[0]);
    TEST_ASSERT_EQUAL_INT32(INT32_MAX, values[1]);
    TEST_ASSERT_FALSE(MotorBoardFrameParser::parseFour(
        "1,2,3,4!trailing", values, INT32_MIN, INT32_MAX
    ));
    TEST_ASSERT_FALSE(MotorBoardFrameParser::parseFour(
        "1,2,3,2147483648!", values, INT32_MIN, INT32_MAX
    ));
    TEST_ASSERT_FALSE(MotorBoardFrameParser::parseFour(
        "1,2,3,32768!", values, -32768, 32767
    ));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_mecanum_forward_and_rotation_geometry);
    RUN_TEST(test_mecanum_proportional_scaling_preserves_ratios);
    RUN_TEST(test_arm_reachable_and_unreachable);
    RUN_TEST(test_scheduler_uses_accumulated_deadlines_without_burst);
    RUN_TEST(test_lightweight_deadline_skips_missed_periods);
    RUN_TEST(test_chassis_ramp_and_controlled_zero_crossing);
    RUN_TEST(test_force_zero_bypasses_ramp);
    RUN_TEST(test_robot_state_and_protocol_values_are_stable);
    RUN_TEST(test_ready_to_arm_requires_every_authoritative_input);
    RUN_TEST(
        test_calibration_actions_require_platform_and_drive_initialization
    );
    RUN_TEST(test_arm_requires_ready_then_link_loss_disarms_immediately);
    RUN_TEST(test_simultaneous_disarm_and_arm_stays_disarmed);
    RUN_TEST(test_fault_requires_explicit_clear_and_never_auto_arms);
    RUN_TEST(test_urgent_frame_is_disarm_and_reserved_generic_is_ignored);
    RUN_TEST(test_held_button_cannot_neutral_qualify_for_arm);
    RUN_TEST(test_calibration_servo_reference_validation_is_atomic);
    RUN_TEST(test_runtime_defaults_keep_calibrated_motor_mapping);
    RUN_TEST(test_motor_parser_resynchronizes_and_bounds_frames);
    RUN_TEST(test_motor_numeric_parser_rejects_malformed_and_out_of_range_values);
    return UNITY_END();
}
