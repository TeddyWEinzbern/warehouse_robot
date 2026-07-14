#include <unity.h>

#include "core/MotorBoardProtocol.h"
#include "core/Scheduler.h"
#include "domain/ArmKinematics.h"
#include "domain/MecanumKinematics.h"
#include "domain/RuntimeConfig.h"
#include "drivers/DriveBackend.h"
#include "subsystems/ChassisSubsystem.h"
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
    void onEncoderTotalDeadline(uint32_t) {}
    void onBatteryDeadline(uint32_t) {}
    void stop(uint32_t) { ++stops; stored = {0, 0, 0, 0}; }
    DriveCapabilities capabilities() const {
        return {DriveControlMode::None, PwmUnit::Unavailable, false, false, false};
    }
    const DriveFeedback &feedback() const { return driveFeedback; }
    DriveHealth health(uint32_t) const { return {0, 0, true, true, true}; }
    void clearFaults() {}
    uint16_t queryTimeouts() const { return 0; }
    uint16_t rxOverflows() const { return 0; }
    uint16_t motorCommandAgeMs(uint32_t) const { return 0; }
    uint8_t outstandingQuery() const { return 0; }
    uint16_t outstandingQueryAgeMs(uint32_t) const { return 0; }
};

OperatorControlFrame neutralFrame(uint32_t receivedAtMs) {
    OperatorControlFrame frame = {};
    frame.valid = true;
    frame.receivedAtMs = receivedAtMs;
    return frame;
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

void test_chassis_ramp_and_controlled_zero_crossing() {
    MockDriveBackend backend;
    ChassisSubsystem chassis(backend);
    RuntimeConfig runtime = RuntimeConfig::defaults();
    runtime.chassis.activeProfile = ResponseProfile::Normal;
    chassis.setDesired({1000, 0, 0, 1000, IntentSource::Operator}, runtime);
    chassis.trajectoryTick(10000, 10000, runtime);
    TEST_ASSERT_EQUAL_INT16(5, chassis.rampedVelocity().longitudinalMmS);
    for (uint32_t now = 20000; now <= 200000; now += 10000)
        chassis.trajectoryTick(now, 10000, runtime);
    TEST_ASSERT_EQUAL_INT16(100, chassis.rampedVelocity().longitudinalMmS);

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

void test_safety_requires_neutral_then_times_out_immediately() {
    SafetySupervisor safety;
    OperatorControlFrame frame = neutralFrame(0);
    DriveHealth healthy = {0, 0, true, true, true};
    safety.update(frame, {0}, healthy, true, true, 0);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(RobotState::Disarmed),
                            static_cast<uint8_t>(safety.state()));
    frame.receivedAtMs = 500;
    safety.update(frame, {RequestArm}, healthy, true, true, 500);
    TEST_ASSERT_TRUE(safety.armed());
    frame.receivedAtMs = 500;
    safety.update(frame, {0}, healthy, true, true, 801);
    TEST_ASSERT_FALSE(safety.armed());
    TEST_ASSERT_TRUE(safety.takeImmediateStop());
}

void test_estop_is_level_triggered_and_clear_never_arms() {
    SafetySupervisor safety;
    OperatorControlFrame frame = neutralFrame(0);
    DriveHealth healthy = {0, 0, true, true, true};
    safety.update(frame, {0}, healthy, true, true, 0);
    frame.controlFlags = EStopAsserted;
    frame.receivedAtMs = 10;
    safety.update(frame, {0}, healthy, true, true, 10);
    TEST_ASSERT_TRUE(safety.emergencyStopped());
    frame.controlFlags = 0;
    frame.receivedAtMs = 100;
    safety.update(frame, {0}, healthy, true, true, 100);
    frame.receivedAtMs = 600;
    safety.update(frame, {RequestClearEStop}, healthy, true, true, 600);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(RobotState::Disarmed),
                            static_cast<uint8_t>(safety.state()));
}

void test_runtime_parameter_commit_is_atomic() {
    RuntimeConfig runtime = RuntimeConfig::defaults();
    const uint16_t revision = runtime.revision;
    const uint8_t invalidServo[4] = {170, 10, 0, 1};
    TEST_ASSERT_FALSE(runtime.applyParameter(
        ParameterGroup::Servo, 0, invalidServo, sizeof(invalidServo), false
    ));
    TEST_ASSERT_EQUAL_UINT16(revision, runtime.revision);
    TEST_ASSERT_EQUAL_UINT8(0, runtime.servos[0].lowerDegrees);
    const uint8_t lowProfile = 0;
    TEST_ASSERT_TRUE(runtime.applyParameter(
        ParameterGroup::ResponseProfile, 0, &lowProfile, 1, false
    ));
    TEST_ASSERT_EQUAL_UINT16(revision + 1, runtime.revision);
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
    int32_t battery = 0;
    TEST_ASSERT_TRUE(MotorBoardFrameParser::parseOne("65535!", battery, 0, 65535));
    TEST_ASSERT_FALSE(MotorBoardFrameParser::parseOne("65536!", battery, 0, 65535));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_mecanum_forward_and_rotation_geometry);
    RUN_TEST(test_mecanum_proportional_scaling_preserves_ratios);
    RUN_TEST(test_arm_reachable_and_unreachable);
    RUN_TEST(test_scheduler_uses_accumulated_deadlines_without_burst);
    RUN_TEST(test_chassis_ramp_and_controlled_zero_crossing);
    RUN_TEST(test_force_zero_bypasses_ramp);
    RUN_TEST(test_safety_requires_neutral_then_times_out_immediately);
    RUN_TEST(test_estop_is_level_triggered_and_clear_never_arms);
    RUN_TEST(test_runtime_parameter_commit_is_atomic);
    RUN_TEST(test_motor_parser_resynchronizes_and_bounds_frames);
    RUN_TEST(test_motor_numeric_parser_rejects_malformed_and_out_of_range_values);
    return UNITY_END();
}
