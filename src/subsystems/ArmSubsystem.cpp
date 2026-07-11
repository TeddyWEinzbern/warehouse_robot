#include "subsystems/ArmSubsystem.h"

#include "app/BuildConfig.h"
#include "app/PinProfile.h"
#include "domain/ArmKinematics.h"

namespace robot {
namespace {
float absFloat(float value) { return value < 0.0F ? -value : value; }
float moveToward(float value, float target, float step) {
    if (target > value + step) return value + step;
    if (target < value - step) return value - step;
    return target;
}
uint8_t clampDegrees(float value) {
    if (value < 0.0F) return 0;
    if (value > 180.0F) return 180;
    return static_cast<uint8_t>(value + 0.5F);
}
}

ArmSubsystem::ArmSubsystem()
    : current_({90.0F, config::StowReachMm, config::StowHeightMm, config::GripperOpenDegrees}),
      goal_(current_), waypointCount_(0), waypointIndex_(0), cargoMayBeHeld_(false),
      gripperCommandDegrees_(config::GripperOpenDegrees), lastUpdateMs_(0) {}

void ArmSubsystem::begin() {
    if (!calibrated()) return;
    servos_[0].attach(pins::ServoBase);
    servos_[1].attach(pins::ServoShoulder);
    servos_[2].attach(pins::ServoElbow);
    servos_[3].attach(pins::ServoGripper);
    applyServos();
}

float ArmSubsystem::clampFloat(float value, float low, float high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

void ArmSubsystem::applyServos() {
    const JointSolution solution = ArmKinematics::solvePlanar(
        current_.reachMm,
        current_.heightMm - config::ShoulderBaseHeightMm,
        config::FirstLinkMm,
        config::SecondLinkMm
    );
    servos_[0].write(clampDegrees(config::BaseZeroDegrees + current_.yawDegrees - 90.0F));
    if (solution.reachable) {
        servos_[1].write(clampDegrees(config::ShoulderZeroDegrees + solution.shoulderDegrees));
        servos_[2].write(clampDegrees(config::ElbowZeroDegrees + solution.elbowDegrees));
    }
    servos_[3].write(current_.gripperDegrees);
}

bool ArmSubsystem::stepTowardGoal(float elapsedSeconds) {
    const float angularStep = 55.0F * elapsedSeconds;
    const float linearStep = 65.0F * elapsedSeconds;
    current_.yawDegrees = moveToward(current_.yawDegrees, goal_.yawDegrees, angularStep);
    current_.reachMm = moveToward(current_.reachMm, goal_.reachMm, linearStep);
    current_.heightMm = moveToward(current_.heightMm, goal_.heightMm, linearStep);
    current_.gripperDegrees = static_cast<uint8_t>(moveToward(
        current_.gripperDegrees, goal_.gripperDegrees, angularStep
    ));
    return absFloat(current_.yawDegrees - goal_.yawDegrees) < 0.5F &&
           absFloat(current_.reachMm - goal_.reachMm) < 0.5F &&
           absFloat(current_.heightMm - goal_.heightMm) < 0.5F &&
           current_.gripperDegrees == goal_.gripperDegrees;
}

void ArmSubsystem::planTo(const ArmTarget &target, bool stowing) {
    waypointCount_ = 0;
    waypointIndex_ = 0;
    const bool needsClearance = cargoMayBeHeld_ &&
        (stowing || current_.heightMm < config::CargoClearanceHeightMm);
    const bool directSafe = !needsClearance &&
        current_.heightMm >= config::MinHeightMm && current_.reachMm >= config::MinReachMm;
    if (config::ActivePresetPolicy == config::PresetPolicy::ViaSafePose || !directSafe) {
        ArmTarget safe = current_;
        safe.heightMm = config::CargoClearanceHeightMm;
        waypoints_[waypointCount_++] = safe;
        if (stowing) {
            safe.reachMm = config::StowReachMm;
            waypoints_[waypointCount_++] = safe;
        }
    }
    waypoints_[waypointCount_++] = target;
    goal_ = waypoints_[0];
}

void ArmSubsystem::requestPreset(uint16_t pressedButtons) {
    if (!calibrated()) return;
    ArmTarget target = current_;
    bool stowing = false;
    if (pressedButtons & PresetLeft) target = {0.0F, config::PresetReachMm, config::PresetHeightMm, current_.gripperDegrees};
    else if (pressedButtons & PresetFront) target = {90.0F, config::PresetReachMm, config::PresetHeightMm, current_.gripperDegrees};
    else if (pressedButtons & PresetRight) target = {180.0F, config::PresetReachMm, config::PresetHeightMm, current_.gripperDegrees};
    else if (pressedButtons & PresetStow) {
        target = {90.0F, config::StowReachMm, config::StowHeightMm, current_.gripperDegrees};
        stowing = true;
    } else return;
    planTo(target, stowing);
}

void ArmSubsystem::requestReach(float reachMm) {
    if (!calibrated()) return;
    waypointCount_ = 0;
    goal_ = current_;
    goal_.reachMm = clampFloat(reachMm, config::MinReachMm, config::MaxReachMm);
}

void ArmSubsystem::update(const OperatorControlFrame &frame, uint32_t nowMs) {
    if (!calibrated()) return;
    uint32_t elapsedMs = nowMs - lastUpdateMs_;
    if (elapsedMs > 100) elapsedMs = 100;
    const float elapsed = elapsedMs / 1000.0F;
    lastUpdateMs_ = nowMs;

    if (calibrated() &&
        (abs(frame.armYaw) > 30 || abs(frame.armReach) > 30 || abs(frame.armHeight) > 30)) {
        waypointCount_ = 0;
        goal_ = current_;
        goal_.yawDegrees = clampFloat(
            goal_.yawDegrees + frame.armYaw / 1000.0F * config::ManualYawDegreesPerSecond * elapsed,
            0.0F, 180.0F
        );
        goal_.reachMm = clampFloat(
            goal_.reachMm + frame.armReach / 1000.0F * config::ManualLinearMmPerSecond * elapsed,
            config::MinReachMm, config::MaxReachMm
        );
        goal_.heightMm = clampFloat(
            goal_.heightMm + frame.armHeight / 1000.0F * config::ManualLinearMmPerSecond * elapsed,
            config::MinHeightMm, config::MaxHeightMm
        );
    }
    if (frame.gripper != 0) {
        const int direction = frame.gripper > 0 ? -1 : 1;
        gripperCommandDegrees_ = clampFloat(
            gripperCommandDegrees_ + direction * 60.0F * elapsed,
            config::GripperClosedDegrees, config::GripperOpenDegrees
        );
        goal_.gripperDegrees = static_cast<uint8_t>(gripperCommandDegrees_ + 0.5F);
        if (goal_.gripperDegrees <= config::GripperClosedDegrees + 3) cargoMayBeHeld_ = true;
        if (goal_.gripperDegrees >= config::GripperOpenDegrees - 3) cargoMayBeHeld_ = false;
    }

    if (stepTowardGoal(elapsed) && waypointCount_ > 0) {
        ++waypointIndex_;
        if (waypointIndex_ < waypointCount_) goal_ = waypoints_[waypointIndex_];
        else waypointCount_ = 0;
    }
    applyServos();
}

void ArmSubsystem::setCalibrationJoint(uint8_t joint, uint8_t degrees) {
    if (joint < 4) {
        servos_[joint].write(degrees > 180 ? 180 : degrees);
        if (joint == 3) gripperCommandDegrees_ = degrees;
    }
}

const ArmTarget &ArmSubsystem::currentTarget() const { return current_; }
bool ArmSubsystem::cargoMayBeHeld() const { return cargoMayBeHeld_; }
bool ArmSubsystem::calibrated() const { return config::ArmCalibrated || ROBOT_ARM_CALIBRATION; }

} // namespace robot
