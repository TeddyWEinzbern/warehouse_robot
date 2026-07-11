#include <unity.h>

#include "domain/ArmKinematics.h"
#include "domain/MecanumKinematics.h"
#include "subsystems/SafetySupervisor.h"

using namespace robot;

void test_mecanum_forward() {
    const WheelTargets wheels = MecanumKinematics::mix(400, 0, 0);
    TEST_ASSERT_EQUAL_INT16(400, wheels.frontLeft);
    TEST_ASSERT_EQUAL_INT16(400, wheels.frontRight);
    TEST_ASSERT_EQUAL_INT16(400, wheels.rearLeft);
    TEST_ASSERT_EQUAL_INT16(400, wheels.rearRight);
}

void test_mecanum_normalizes() {
    const WheelTargets wheels = MecanumKinematics::mix(1000, 1000, 1000);
    TEST_ASSERT_EQUAL_INT16(1000, wheels.frontLeft);
    TEST_ASSERT_TRUE(wheels.frontRight >= -1000 && wheels.frontRight <= 1000);
}

void test_arm_reachable_and_unreachable() {
    TEST_ASSERT_TRUE(ArmKinematics::solvePlanar(100, 80, 110, 110).reachable);
    TEST_ASSERT_FALSE(ArmKinematics::solvePlanar(300, 0, 110, 110).reachable);
}

void test_watchdog_has_priority() {
    SafetySupervisor safety;
    OperatorControlFrame frame = {};
    frame.valid = true;
    frame.forward = 700;
    frame.receivedAtMs = 0;
    AssistOutput assist = {};
    const DriveIntent result = safety.arbitrate(frame, assist, false, 301);
    TEST_ASSERT_EQUAL_INT16(0, result.forward);
    TEST_ASSERT_EQUAL_UINT16(0, result.maxMagnitude);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_mecanum_forward);
    RUN_TEST(test_mecanum_normalizes);
    RUN_TEST(test_arm_reachable_and_unreachable);
    RUN_TEST(test_watchdog_has_priority);
    return UNITY_END();
}

