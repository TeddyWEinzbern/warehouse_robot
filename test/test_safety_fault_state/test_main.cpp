#include <unity.h>

#include "app/BuildConfig.h"
#include "domain/RobotTypes.h"
#include "subsystems/SafetySupervisor.h"

using namespace robot;

namespace {
OperatorControlFrame neutralFrame(uint32_t nowMs) {
    return {
        0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, nowMs, true
    };
}

DriveHealth healthyDrive(uint16_t faults = 0) {
    return {faults, 0, true, true, true};
}

void qualifyAndArm(SafetySupervisor &safety) {
    OperatorControlFrame frame = neutralFrame(0);
    safety.update(frame, {0}, healthyDrive(), true, true, 0);
    frame.receivedAtMs = 500;
    safety.update(frame, {RequestArm}, healthyDrive(), true, true, 500);
    TEST_ASSERT_TRUE(safety.armed());
    safety.takeImmediateStop();
}
}

void test_every_fault_bit_is_latched_and_unsafe_only_bypasses_fault_state() {
    SafetySupervisor safety;
    qualifyAndArm(safety);
    const uint16_t allFaults =
        FaultSchedulerOverrun | FaultDriveInitialization |
        FaultEncoderStale | FaultEncoderMalformed |
        FaultEncoderImplausible | FaultEncoderSign |
        FaultDriveStall | FaultDriveMismatch | FaultArmTarget;

    OperatorControlFrame frame = neutralFrame(501);
    safety.update(
        frame, {0}, healthyDrive(allFaults), true, true, 501
    );

    TEST_ASSERT_EQUAL_HEX16(allFaults, safety.faults());
#if ROBOT_FAULT_STATE_UNSAFE
    TEST_ASSERT_TRUE(safety.armed());
    TEST_ASSERT_FALSE(safety.takeImmediateStop());
#else
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(RobotState::Fault),
        static_cast<uint8_t>(safety.state())
    );
    TEST_ASSERT_TRUE(safety.takeImmediateStop());
#endif
}

void test_latch_fault_uses_the_same_unsafe_state_policy() {
    SafetySupervisor safety;
    qualifyAndArm(safety);

    safety.latchFault(FaultSchedulerOverrun | FaultArmTarget);

    TEST_ASSERT_EQUAL_HEX16(
        FaultSchedulerOverrun | FaultArmTarget, safety.faults()
    );
#if ROBOT_FAULT_STATE_UNSAFE
    TEST_ASSERT_TRUE(safety.armed());
    TEST_ASSERT_FALSE(safety.takeImmediateStop());
#else
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(RobotState::Fault),
        static_cast<uint8_t>(safety.state())
    );
    TEST_ASSERT_TRUE(safety.takeImmediateStop());
#endif
}

void test_unsafe_disarmed_faults_can_be_explicitly_cleared() {
    SafetySupervisor safety;
    OperatorControlFrame frame = neutralFrame(0);
    safety.update(
        frame, {0}, healthyDrive(FaultDriveStall), true, true, 0
    );

#if ROBOT_FAULT_STATE_UNSAFE
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(RobotState::Disarmed),
        static_cast<uint8_t>(safety.state())
    );
#else
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(RobotState::Fault),
        static_cast<uint8_t>(safety.state())
    );
#endif

    frame.receivedAtMs = 500;
    safety.update(
        frame, {RequestClearFault}, healthyDrive(),
        true, true, 500
    );
    TEST_ASSERT_EQUAL_UINT16(0, safety.faults());
    TEST_ASSERT_TRUE(safety.takeClearFaultAccepted());
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(RobotState::Disarmed),
        static_cast<uint8_t>(safety.state())
    );
}

void test_unsafe_fault_bypass_does_not_bypass_disarm_or_link_loss() {
    SafetySupervisor safety;
    qualifyAndArm(safety);
    OperatorControlFrame frame = neutralFrame(501);
    safety.update(
        frame, {RequestDisarm}, healthyDrive(FaultDriveStall),
        true, true, 501
    );
    TEST_ASSERT_FALSE(safety.armed());
    TEST_ASSERT_TRUE(safety.takeImmediateStop());
#if ROBOT_FAULT_STATE_UNSAFE
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(RobotState::Disarmed),
        static_cast<uint8_t>(safety.state())
    );
    frame = neutralFrame(1001);
    safety.update(
        frame, {RequestArm}, healthyDrive(), true, true, 1001
    );
    TEST_ASSERT_FALSE(safety.armed());
    TEST_ASSERT_FALSE(safety.readyToArm());
#else
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(RobotState::Fault),
        static_cast<uint8_t>(safety.state())
    );
#endif

    SafetySupervisor linkSafety;
    qualifyAndArm(linkSafety);
    frame = neutralFrame(0);
    linkSafety.update(
        frame, {0}, healthyDrive(FaultDriveStall), true, true, 801
    );
#if ROBOT_FAULT_STATE_UNSAFE
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(RobotState::Disarmed),
        static_cast<uint8_t>(linkSafety.state())
    );
#else
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(RobotState::Fault),
        static_cast<uint8_t>(linkSafety.state())
    );
#endif
}

void test_clear_fault_requires_recovered_feedback() {
    SafetySupervisor safety;
    OperatorControlFrame frame = neutralFrame(0);
    const DriveHealth staleButBypassed = {
        FaultEncoderStale,
        static_cast<uint16_t>(
            WarningFaultStateUnsafe | WarningEncoderTimeoutIgnored
        ),
        true, true, true
    };
    safety.update(
        frame, {0}, staleButBypassed, true, true, 0
    );
    frame.receivedAtMs = 500;
    safety.update(
        frame, {RequestClearFault}, staleButBypassed,
        true, true, 500
    );
    TEST_ASSERT_EQUAL_HEX16(FaultEncoderStale, safety.faults());
    TEST_ASSERT_FALSE(safety.takeClearFaultAccepted());

    frame.receivedAtMs = 501;
    safety.update(
        frame, {RequestClearFault}, healthyDrive(),
        true, true, 501
    );
    TEST_ASSERT_EQUAL_UINT16(0, safety.faults());
    TEST_ASSERT_TRUE(safety.takeClearFaultAccepted());
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(
        test_every_fault_bit_is_latched_and_unsafe_only_bypasses_fault_state
    );
    RUN_TEST(test_latch_fault_uses_the_same_unsafe_state_policy);
    RUN_TEST(test_unsafe_disarmed_faults_can_be_explicitly_cleared);
    RUN_TEST(
        test_unsafe_fault_bypass_does_not_bypass_disarm_or_link_loss
    );
    RUN_TEST(test_clear_fault_requires_recovered_feedback);
    return UNITY_END();
}
