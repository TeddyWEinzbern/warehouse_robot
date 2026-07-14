#include "subsystems/ArmSubsystem.h"

#include <math.h>

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
} // namespace

ArmSubsystem::ArmSubsystem()
    : current_({
          90.0F, config::StowReachMm, config::StowHeightMm,
          config::GripperOpenDegrees
      }),
      goal_(current_), waypointCount_(0), waypointIndex_(0),
      cargoMayBeHeld_(false), holding_(false), faulted_(false),
      currentGripperDegrees_(config::GripperOpenDegrees),
      goalGripperDegrees_(config::GripperOpenDegrees),
      lastCommandedDegrees_{0, 0, 0, 0} {}

void ArmSubsystem::begin(const RuntimeConfig &runtime) {
    if (!calibrated()) return;
    current_ = {
        90.0F, static_cast<float>(runtime.arm.stowReachMm),
        static_cast<float>(runtime.arm.stowHeightMm), config::GripperOpenDegrees
    };
    goal_ = current_;
    currentGripperDegrees_ = goalGripperDegrees_ = config::GripperOpenDegrees;
    servos_[0].attach(pins::ServoBase);
    servos_[1].attach(pins::ServoShoulder);
    servos_[2].attach(pins::ServoElbow);
    servos_[3].attach(pins::ServoGripper);
    faulted_ = !applyServos(runtime);
}

float ArmSubsystem::clampFloat(float value, float low, float high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

uint8_t ArmSubsystem::mapJoint(
    uint8_t joint, float offsetFromCenter, const RuntimeConfig &runtime
) const {
    const ServoCalibration &calibration = runtime.servos[joint];
    const float center = 90.0F + calibration.centerOffsetDegrees;
    return clampDegrees(clampFloat(
        center + calibration.direction * offsetFromCenter,
        calibration.lowerDegrees,
        calibration.upperDegrees
    ));
}

bool ArmSubsystem::applyServos(const RuntimeConfig &runtime) {
    const JointSolution solution = ArmKinematics::solvePlanar(
        current_.reachMm,
        current_.heightMm - runtime.arm.shoulderBaseHeightMm,
        runtime.arm.firstLinkMm,
        runtime.arm.secondLinkMm
    );
    if (!solution.reachable || !isfinite(solution.shoulderDegrees) ||
        !isfinite(solution.elbowDegrees)) return false;

    const uint8_t commands[4] = {
        mapJoint(0, current_.yawDegrees - 90.0F, runtime),
        mapJoint(1, solution.shoulderDegrees, runtime),
        mapJoint(2, solution.elbowDegrees, runtime),
        mapJoint(3, currentGripperDegrees_ - config::GripperOpenDegrees, runtime),
    };
    for (uint8_t joint = 0; joint < 4; ++joint) {
        servos_[joint].write(commands[joint]);
        lastCommandedDegrees_[joint] = commands[joint];
    }
    return true;
}

bool ArmSubsystem::stepTowardGoal(float elapsedSeconds) {
    const float angularStep = 55.0F * elapsedSeconds;
    const float linearStep = 65.0F * elapsedSeconds;
    current_.yawDegrees = moveToward(current_.yawDegrees, goal_.yawDegrees, angularStep);
    current_.reachMm = moveToward(current_.reachMm, goal_.reachMm, linearStep);
    current_.heightMm = moveToward(current_.heightMm, goal_.heightMm, linearStep);
    currentGripperDegrees_ = moveToward(
        currentGripperDegrees_, goalGripperDegrees_, angularStep
    );
    current_.gripperDegrees = clampDegrees(currentGripperDegrees_);
    return absFloat(current_.yawDegrees - goal_.yawDegrees) < 0.5F &&
           absFloat(current_.reachMm - goal_.reachMm) < 0.5F &&
           absFloat(current_.heightMm - goal_.heightMm) < 0.5F &&
           absFloat(currentGripperDegrees_ - goalGripperDegrees_) < 0.5F;
}

void ArmSubsystem::planTo(
    const ArmTarget &target, bool stowing, const RuntimeConfig &runtime
) {
    waypointCount_ = 0;
    waypointIndex_ = 0;
    const bool needsClearance = cargoMayBeHeld_ &&
        (stowing || current_.heightMm < runtime.arm.cargoClearanceHeightMm);
    const bool directSafe = !needsClearance &&
        current_.heightMm >= runtime.arm.minimumHeightMm &&
        current_.reachMm >= runtime.arm.minimumReachMm;
    if (config::ActivePresetPolicy == config::PresetPolicy::ViaSafePose ||
        !directSafe) {
        ArmTarget safe = current_;
        safe.heightMm = runtime.arm.cargoClearanceHeightMm;
        waypoints_[waypointCount_++] = safe;
        if (stowing) {
            safe.reachMm = runtime.arm.stowReachMm;
            waypoints_[waypointCount_++] = safe;
        }
    }
    waypoints_[waypointCount_++] = target;
    goal_ = waypoints_[0];
    goalGripperDegrees_ = goal_.gripperDegrees;
}

void ArmSubsystem::requestPreset(
    uint16_t pressedButtons, const RuntimeConfig &runtime
) {
    if (!calibrated()) return;
    ArmTarget target = current_;
    bool stowing = false;
    if (pressedButtons & PresetLeft) {
        target = {0.0F, static_cast<float>(runtime.arm.presetReachMm),
                  static_cast<float>(runtime.arm.presetHeightMm),
                  current_.gripperDegrees};
    } else if (pressedButtons & PresetFront) {
        target = {90.0F, static_cast<float>(runtime.arm.presetReachMm),
                  static_cast<float>(runtime.arm.presetHeightMm),
                  current_.gripperDegrees};
    } else if (pressedButtons & PresetRight) {
        target = {180.0F, static_cast<float>(runtime.arm.presetReachMm),
                  static_cast<float>(runtime.arm.presetHeightMm),
                  current_.gripperDegrees};
    } else if (pressedButtons & PresetStow) {
        target = {90.0F, static_cast<float>(runtime.arm.stowReachMm),
                  static_cast<float>(runtime.arm.stowHeightMm),
                  current_.gripperDegrees};
        stowing = true;
    } else {
        return;
    }
    holding_ = false;
    planTo(target, stowing, runtime);
}

void ArmSubsystem::requestReach(float reachMm, const RuntimeConfig &runtime) {
    if (!calibrated()) return;
    holding_ = false;
    waypointCount_ = 0;
    goal_ = current_;
    goal_.reachMm = clampFloat(
        reachMm, runtime.arm.minimumReachMm, runtime.arm.maximumReachMm
    );
    goalGripperDegrees_ = goal_.gripperDegrees;
}

void ArmSubsystem::update(
    const OperatorControlFrame &frame,
    uint32_t elapsedUs,
    const RuntimeConfig &runtime
) {
    if (!calibrated() || holding_) return;
    const ArmTarget previous = current_;
    const float previousGripper = currentGripperDegrees_;
    if (elapsedUs > config::MaxMotionDtUs) elapsedUs = config::MaxMotionDtUs;
    const float elapsed = elapsedUs / 1000000.0F;

    if (abs(frame.armYaw) > 30 || abs(frame.armReach) > 30 ||
        abs(frame.armHeight) > 30) {
        waypointCount_ = 0;
        goal_ = current_;
        goal_.yawDegrees = clampFloat(
            goal_.yawDegrees + frame.armYaw / 1000.0F *
                config::ManualYawDegreesPerSecond * elapsed,
            0.0F, 180.0F
        );
        goal_.reachMm = clampFloat(
            goal_.reachMm + frame.armReach / 1000.0F *
                config::ManualLinearMmPerSecond * elapsed,
            runtime.arm.minimumReachMm, runtime.arm.maximumReachMm
        );
        goal_.heightMm = clampFloat(
            goal_.heightMm + frame.armHeight / 1000.0F *
                config::ManualLinearMmPerSecond * elapsed,
            runtime.arm.minimumHeightMm, runtime.arm.maximumHeightMm
        );
    }
    if (frame.gripper != 0) {
        const int direction = frame.gripper > 0 ? -1 : 1;
        goalGripperDegrees_ = clampFloat(
            goalGripperDegrees_ + direction * 60.0F * elapsed,
            config::GripperClosedDegrees, config::GripperOpenDegrees
        );
        goal_.gripperDegrees = clampDegrees(goalGripperDegrees_);
        if (goal_.gripperDegrees <= config::GripperClosedDegrees + 3)
            cargoMayBeHeld_ = true;
        if (goal_.gripperDegrees >= config::GripperOpenDegrees - 3)
            cargoMayBeHeld_ = false;
    }

    if (stepTowardGoal(elapsed) && waypointCount_ > 0) {
        ++waypointIndex_;
        if (waypointIndex_ < waypointCount_) {
            goal_ = waypoints_[waypointIndex_];
            goalGripperDegrees_ = goal_.gripperDegrees;
        } else {
            waypointCount_ = 0;
        }
    }
    faulted_ = !applyServos(runtime);
    if (faulted_) {
        current_ = previous;
        currentGripperDegrees_ = previousGripper;
        holdLastCommanded();
    }
}

void ArmSubsystem::setCalibrationJoint(uint8_t joint, uint8_t degrees) {
    if (joint >= 4 || !calibrated()) return;
    const uint8_t bounded = degrees > 180 ? 180 : degrees;
    servos_[joint].write(bounded);
    lastCommandedDegrees_[joint] = bounded;
    if (joint == 3) {
        currentGripperDegrees_ = bounded;
        goalGripperDegrees_ = bounded;
    }
}

void ArmSubsystem::holdLastCommanded() {
    waypointCount_ = 0;
    waypointIndex_ = 0;
    goal_ = current_;
    goalGripperDegrees_ = currentGripperDegrees_;
    holding_ = true;
}

void ArmSubsystem::releaseHold() { holding_ = false; }
void ArmSubsystem::clearFault() {
    faulted_ = false;
    goal_ = current_;
    goalGripperDegrees_ = currentGripperDegrees_;
}

const ArmTarget &ArmSubsystem::currentTarget() const { return current_; }
const uint8_t *ArmSubsystem::lastCommandedDegrees() const {
    return lastCommandedDegrees_;
}
bool ArmSubsystem::cargoMayBeHeld() const { return cargoMayBeHeld_; }
bool ArmSubsystem::calibrated() const {
    return config::ArmCalibrated || ROBOT_ARM_CALIBRATION;
}
bool ArmSubsystem::faulted() const { return faulted_; }

} // namespace robot
