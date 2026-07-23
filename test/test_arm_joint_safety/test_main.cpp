#include <unity.h>

#include <math.h>

#include "app/BuildConfig.h"
#include "domain/ArmKinematics.h"
#include "domain/RuntimeConfig.h"

using namespace robot;

namespace {
PlanarLimits configuredAnnulus() {
    return ArmKinematics::radialLimits(
        config::FirstLinkMm, config::SecondLinkMm,
        config::ElbowFoldMinDegrees, config::ElbowFoldMaxDegrees
    );
}
} // namespace

void test_solver_returns_physical_elbow_up_branch() {
    const JointSolution solution = ArmKinematics::solvePlanar(120, 120, 120, 120);
    TEST_ASSERT_TRUE(solution.reachable);
    TEST_ASSERT_FLOAT_WITHIN(0.1F, 90.0F, solution.shoulderDegrees);
    TEST_ASSERT_FLOAT_WITHIN(0.1F, 90.0F, solution.elbowDegrees);
    TEST_ASSERT_FLOAT_WITHIN(
        0.1F, 0.0F, solution.shoulderDegrees - solution.elbowDegrees
    );
}

void test_old_box_corner_folds_past_the_four_bar_limit() {
    const JointSolution solution = ArmKinematics::solvePlanar(55, -20, 120, 120);
    TEST_ASSERT_TRUE(solution.reachable);
    TEST_ASSERT_TRUE(solution.elbowDegrees > config::ElbowFoldMaxDegrees);
}

void test_radial_limits_match_fold_band() {
    const PlanarLimits limits = ArmKinematics::radialLimits(120, 120, 5, 135);
    TEST_ASSERT_FLOAT_WITHIN(0.5F, 91.8F, limits.minRadialMm);
    TEST_ASSERT_FLOAT_WITHIN(0.5F, 239.8F, limits.maxRadialMm);
}

void test_constrain_projects_onto_annulus_preserving_direction() {
    const PlanarLimits limits = configuredAnnulus();
    float reach = 55.0F;
    float height = -20.0F;
    TEST_ASSERT_TRUE(ArmKinematics::constrainPlanar(reach, height, limits));
    const float distance = sqrtf(reach * reach + height * height);
    TEST_ASSERT_FLOAT_WITHIN(0.1F, limits.minRadialMm, distance);
    TEST_ASSERT_TRUE(reach > 0.0F && height < 0.0F);
    TEST_ASSERT_FLOAT_WITHIN(0.02F, -20.0F / 55.0F, height / reach);

    const JointSolution solution = ArmKinematics::solvePlanar(
        reach, height, config::FirstLinkMm, config::SecondLinkMm
    );
    TEST_ASSERT_TRUE(solution.reachable);
    TEST_ASSERT_TRUE(
        solution.elbowDegrees <= config::ElbowFoldMaxDegrees + 0.5F
    );
}

void test_constrain_leaves_interior_point_untouched() {
    const PlanarLimits limits = configuredAnnulus();
    float reach = 135.0F;
    float height = 50.0F;
    TEST_ASSERT_FALSE(ArmKinematics::constrainPlanar(reach, height, limits));
    TEST_ASSERT_EQUAL_FLOAT(135.0F, reach);
    TEST_ASSERT_EQUAL_FLOAT(50.0F, height);
}

void test_runtime_defaults_match_build_geometry() {
    const RuntimeConfig runtime = RuntimeConfig::defaults();
    TEST_ASSERT_EQUAL_UINT16(
        static_cast<uint16_t>(config::FirstLinkMm), runtime.arm.firstLinkMm
    );
    TEST_ASSERT_EQUAL_UINT16(
        static_cast<uint16_t>(config::SecondLinkMm), runtime.arm.secondLinkMm
    );
    TEST_ASSERT_EQUAL_UINT16(120, runtime.arm.firstLinkMm);
    TEST_ASSERT_EQUAL_UINT16(120, runtime.arm.secondLinkMm);
}

void test_default_presets_and_waypoints_stay_inside_fold_band() {
    const RuntimeConfig runtime = RuntimeConfig::defaults();
    const float base = runtime.arm.shoulderBaseHeightMm;
    const float poses[4][2] = {
        {static_cast<float>(runtime.arm.presetReachMm),
         static_cast<float>(runtime.arm.presetHeightMm)},
        {static_cast<float>(runtime.arm.stowReachMm),
         static_cast<float>(runtime.arm.stowHeightMm)},
        {static_cast<float>(runtime.arm.presetReachMm),
         static_cast<float>(runtime.arm.cargoClearanceHeightMm)},
        {static_cast<float>(runtime.arm.stowReachMm),
         static_cast<float>(runtime.arm.cargoClearanceHeightMm)},
    };
    for (uint8_t index = 0; index < 4; ++index) {
        const JointSolution solution = ArmKinematics::solvePlanar(
            poses[index][0], poses[index][1] - base,
            runtime.arm.firstLinkMm, runtime.arm.secondLinkMm
        );
        TEST_ASSERT_TRUE(solution.reachable);
        TEST_ASSERT_TRUE(solution.elbowDegrees <= config::ElbowFoldMaxDegrees);
        TEST_ASSERT_TRUE(
            solution.elbowDegrees >=
            config::ElbowFoldMinDegrees - config::ElbowFoldSlackDegrees
        );
    }
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_solver_returns_physical_elbow_up_branch);
    RUN_TEST(test_old_box_corner_folds_past_the_four_bar_limit);
    RUN_TEST(test_radial_limits_match_fold_band);
    RUN_TEST(test_constrain_projects_onto_annulus_preserving_direction);
    RUN_TEST(test_constrain_leaves_interior_point_untouched);
    RUN_TEST(test_runtime_defaults_match_build_geometry);
    RUN_TEST(test_default_presets_and_waypoints_stay_inside_fold_band);
    return UNITY_END();
}
