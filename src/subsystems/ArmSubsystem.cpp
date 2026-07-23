#include "subsystems/ArmSubsystem.h"

#include <math.h>

#include "app/BuildConfig.h"
#include "app/PinProfile.h"

namespace robot {
namespace {
const uint8_t ServoPins[4] = {
    pins::ServoBase, pins::ServoShoulder, pins::ServoElbow, pins::ServoGripper
};

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

float offsetFromRaw(const ServoCalibration &calibration, float rawDegrees) {
    const float center = 90.0F + calibration.centerOffsetDegrees;
    return calibration.direction >= 0 ? rawDegrees - center : center - rawDegrees;
}

PlanarLimits foldAnnulus(const ArmGeometry &arm) {
    return ArmKinematics::radialLimits(
        arm.firstLinkMm, arm.secondLinkMm,
        config::ElbowFoldMinDegrees, config::ElbowFoldMaxDegrees
    );
}
} // namespace

ArmSubsystem::ArmSubsystem()
    : current_({
          90.0F, config::StowReachMm, config::StowHeightMm,
          config::GripperOpenDegrees
      }),
      goal_(current_), waypointCount_(0), waypointIndex_(0),
      cargoMayBeHeld_(false), holding_(false), faulted_(false),
      targetLimitedTicks_(0), attached_{false, false, false, false},
      jointDegrees_{90.0F, 90.0F, 90.0F, 90.0F},
      calibrationTarget_{90.0F, 90.0F, 90.0F, 90.0F},
      calibrationPending_{false, false, false, false},
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
    // Servos stay detached until the first commanded output (first armed
    // update, or first calibration command), so powering the board never
    // moves the arm on its own.
    faulted_ = false;
}

float ArmSubsystem::clampFloat(float value, float low, float high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

void ArmSubsystem::markLimited() { targetLimitedTicks_ = 100; }

void ArmSubsystem::writeJoint(uint8_t joint, float degrees) {
    if (!attached_[joint]) {
        servos_[joint].attach(ServoPins[joint]);
        attached_[joint] = true;
    }
    const uint8_t rounded = clampDegrees(degrees);
    servos_[joint].write(rounded);
    jointDegrees_[joint] = degrees;
    lastCommandedDegrees_[joint] = rounded;
}

bool ArmSubsystem::mapJoint(
    uint8_t joint, float offsetFromCenter, const RuntimeConfig &runtime,
    uint8_t &outDegrees
) const {
    const ServoCalibration &calibration = runtime.servos[joint];
    const float center = 90.0F + calibration.centerOffsetDegrees;
    const float value = center + calibration.direction * offsetFromCenter;
    if (value < calibration.lowerDegrees - config::ServoClampToleranceDegrees ||
        value > calibration.upperDegrees + config::ServoClampToleranceDegrees)
        return false;
    outDegrees = clampDegrees(clampFloat(
        value, calibration.lowerDegrees, calibration.upperDegrees
    ));
    return true;
}

void ArmSubsystem::jointOffsetRange(
    uint8_t joint, const RuntimeConfig &runtime, float &minOffset,
    float &maxOffset
) const {
    const ServoCalibration &calibration = runtime.servos[joint];
    const float center = 90.0F + calibration.centerOffsetDegrees;
    if (calibration.direction >= 0) {
        minOffset = calibration.lowerDegrees - center;
        maxOffset = calibration.upperDegrees - center;
    } else {
        minOffset = center - calibration.upperDegrees;
        maxOffset = center - calibration.lowerDegrees;
    }
}

bool ArmSubsystem::constrainTarget(
    ArmTarget &target, const RuntimeConfig &runtime
) const {
    bool limited = false;
    const ArmGeometry &arm = runtime.arm;

    float yawLow = 0.0F;
    float yawHigh = 0.0F;
    jointOffsetRange(0, runtime, yawLow, yawHigh);
    const float yaw = clampFloat(
        target.yawDegrees,
        clampFloat(90.0F + yawLow, 0.0F, 180.0F),
        clampFloat(90.0F + yawHigh, 0.0F, 180.0F)
    );
    if (yaw != target.yawDegrees) {
        target.yawDegrees = yaw;
        limited = true;
    }

    float reach = clampFloat(target.reachMm, arm.minimumReachMm, arm.maximumReachMm);
    float height = clampFloat(target.heightMm, arm.minimumHeightMm, arm.maximumHeightMm);
    if (reach != target.reachMm || height != target.heightMm) limited = true;

    // The reach/height box and the fold-angle annulus overlap but neither
    // contains the other; two alternating passes settle close enough that the
    // fold slack in applyServos accepts the result.
    const PlanarLimits radial = foldAnnulus(arm);
    for (uint8_t pass = 0; pass < 2; ++pass) {
        float relative = height - arm.shoulderBaseHeightMm;
        if (ArmKinematics::constrainPlanar(reach, relative, radial)) limited = true;
        height = relative + arm.shoulderBaseHeightMm;
        reach = clampFloat(reach, arm.minimumReachMm, arm.maximumReachMm);
        height = clampFloat(height, arm.minimumHeightMm, arm.maximumHeightMm);
    }
    target.reachMm = reach;
    target.heightMm = height;
    return limited;
}

bool ArmSubsystem::applyServos(const RuntimeConfig &runtime) {
    const JointSolution solution = ArmKinematics::solvePlanar(
        current_.reachMm,
        current_.heightMm - runtime.arm.shoulderBaseHeightMm,
        runtime.arm.firstLinkMm,
        runtime.arm.secondLinkMm
    );
    if (!solution.reachable) return false;
    if (!isfinite(solution.shoulderDegrees) || !isfinite(solution.elbowDegrees)) {
        faulted_ = true;
        return false;
    }
    if (solution.elbowDegrees >
            config::ElbowFoldMaxDegrees + config::ElbowFoldSlackDegrees ||
        solution.elbowDegrees <
            config::ElbowFoldMinDegrees - config::ElbowFoldSlackDegrees)
        return false;

    // The elbow servo is grounded: through the drive parallelogram it sets
    // the forearm's absolute pitch, not the fold angle.
    const float forearmAbsolute = solution.shoulderDegrees - solution.elbowDegrees;
    uint8_t commands[4];
    if (!mapJoint(0, current_.yawDegrees - 90.0F, runtime, commands[0]) ||
        !mapJoint(1, solution.shoulderDegrees - 90.0F, runtime, commands[1]) ||
        !mapJoint(2, forearmAbsolute, runtime, commands[2]) ||
        !mapJoint(
            3, currentGripperDegrees_ - config::GripperOpenDegrees, runtime,
            commands[3]
        ))
        return false;
    for (uint8_t joint = 0; joint < 4; ++joint)
        writeJoint(joint, commands[joint]);
    return true;
}

bool ArmSubsystem::stepTowardGoal(
    float elapsedSeconds, const RuntimeConfig &runtime
) {
    const float angularStep = config::ArmYawDegreesPerSecond * elapsedSeconds;
    current_.yawDegrees = moveToward(current_.yawDegrees, goal_.yawDegrees, angularStep);

    const float deltaReach = goal_.reachMm - current_.reachMm;
    const float deltaHeight = goal_.heightMm - current_.heightMm;
    const float distance =
        sqrtf(deltaReach * deltaReach + deltaHeight * deltaHeight);
    const float linearStep = config::ArmLinearMmPerSecond * elapsedSeconds;
    if (distance <= linearStep || distance < 0.001F) {
        current_.reachMm = goal_.reachMm;
        current_.heightMm = goal_.heightMm;
    } else {
        current_.reachMm += deltaReach / distance * linearStep;
        current_.heightMm += deltaHeight / distance * linearStep;
    }
    // A straight segment between two annulus points can cut through the
    // inner (collision) circle; slide the interpolated point around it.
    float relative = current_.heightMm - runtime.arm.shoulderBaseHeightMm;
    ArmKinematics::constrainPlanar(current_.reachMm, relative, foldAnnulus(runtime.arm));
    current_.heightMm = relative + runtime.arm.shoulderBaseHeightMm;

    currentGripperDegrees_ = moveToward(
        currentGripperDegrees_, goalGripperDegrees_,
        config::GripperDegreesPerSecond * elapsedSeconds
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
        constrainTarget(safe, runtime);
        waypoints_[waypointCount_++] = safe;
        if (stowing) {
            safe.reachMm = runtime.arm.stowReachMm;
            constrainTarget(safe, runtime);
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
    if (constrainTarget(target, runtime)) markLimited();
    const JointSolution probe = ArmKinematics::solvePlanar(
        target.reachMm, target.heightMm - runtime.arm.shoulderBaseHeightMm,
        runtime.arm.firstLinkMm, runtime.arm.secondLinkMm
    );
    if (!probe.reachable) {
        markLimited();
        return;
    }
    holding_ = false;
    planTo(target, stowing, runtime);
}

void ArmSubsystem::requestReach(float reachMm, const RuntimeConfig &runtime) {
    if (!calibrated()) return;
    holding_ = false;
    waypointCount_ = 0;
    waypointIndex_ = 0;
    goal_ = current_;
    goal_.reachMm = reachMm;
    if (constrainTarget(goal_, runtime)) markLimited();
    goalGripperDegrees_ = goal_.gripperDegrees;
}

void ArmSubsystem::update(
    const OperatorControlFrame &frame,
    uint32_t elapsedUs,
    const RuntimeConfig &runtime
) {
    if (!calibrated() || holding_) return;
    if (targetLimitedTicks_ > 0) --targetLimitedTicks_;
    const ArmTarget previous = current_;
    const float previousGripper = currentGripperDegrees_;
    if (elapsedUs > config::MaxMotionDtUs) elapsedUs = config::MaxMotionDtUs;
    const float elapsed = elapsedUs / 1000000.0F;

    if (abs(frame.armYaw) > 30 || abs(frame.armReach) > 30 ||
        abs(frame.armHeight) > 30) {
        waypointCount_ = 0;
        waypointIndex_ = 0;
        goal_ = current_;
        goal_.yawDegrees += frame.armYaw / 1000.0F *
            config::ArmYawDegreesPerSecond * elapsed;
        // Combined reach/height speed stays capped at the linear rate.
        float reachRate = frame.armReach / 1000.0F * config::ArmLinearMmPerSecond;
        float heightRate = frame.armHeight / 1000.0F * config::ArmLinearMmPerSecond;
        const float magnitude =
            sqrtf(reachRate * reachRate + heightRate * heightRate);
        if (magnitude > config::ArmLinearMmPerSecond) {
            const float scale = config::ArmLinearMmPerSecond / magnitude;
            reachRate *= scale;
            heightRate *= scale;
        }
        goal_.reachMm += reachRate * elapsed;
        goal_.heightMm += heightRate * elapsed;
        if (constrainTarget(goal_, runtime)) markLimited();
    }
    if (frame.gripper != 0) {
        const int direction = frame.gripper > 0 ? -1 : 1;
        goalGripperDegrees_ = clampFloat(
            goalGripperDegrees_ +
                direction * config::GripperDegreesPerSecond * elapsed,
            config::GripperClosedDegrees, config::GripperOpenDegrees
        );
        goal_.gripperDegrees = clampDegrees(goalGripperDegrees_);
        if (goal_.gripperDegrees <= config::GripperClosedDegrees + 3)
            cargoMayBeHeld_ = true;
        if (goal_.gripperDegrees >= config::GripperOpenDegrees - 3)
            cargoMayBeHeld_ = false;
    }

    if (stepTowardGoal(elapsed, runtime) && waypointCount_ > 0) {
        ++waypointIndex_;
        if (waypointIndex_ < waypointCount_) {
            goal_ = waypoints_[waypointIndex_];
            goalGripperDegrees_ = goal_.gripperDegrees;
        } else {
            waypointCount_ = 0;
        }
    }
    if (!applyServos(runtime)) {
        // Keep the commanded model exactly where the hardware last was; a
        // silently clamped write would let the planner believe a pose the
        // arm never took.
        current_ = previous;
        currentGripperDegrees_ = previousGripper;
        goal_ = current_;
        goalGripperDegrees_ = currentGripperDegrees_;
        waypointCount_ = 0;
        waypointIndex_ = 0;
        markLimited();
        if (faulted_) holdLastCommanded();
    }
}

bool ArmSubsystem::foldAllowed(
    float shoulderRaw, float elbowRaw, const RuntimeConfig &runtime
) const {
    const float shoulderAbsolute =
        90.0F + offsetFromRaw(runtime.servos[1], shoulderRaw);
    const float forearmAbsolute = offsetFromRaw(runtime.servos[2], elbowRaw);
    const float fold = shoulderAbsolute - forearmAbsolute;
    // Calibration jog may explore slightly past the running band, but never
    // into four-bar flattening.
    return fold >= config::ElbowFoldMinDegrees - 10.0F &&
           fold <= config::ElbowFoldMaxDegrees + config::ElbowFoldSlackDegrees;
}

bool ArmSubsystem::setCalibrationJoint(
    uint8_t joint, uint8_t degrees, const RuntimeConfig &runtime
) {
    if (joint >= 4 || !calibrated()) return false;
    const float bounded = degrees > 180 ? 180.0F : degrees;
    if (joint == 1 || joint == 2) {
        const uint8_t partner = joint == 1 ? 2 : 1;
        if (attached_[partner]) {
            const float partnerDegrees = calibrationPending_[partner]
                ? calibrationTarget_[partner] : jointDegrees_[partner];
            const float shoulderRaw = joint == 1 ? bounded : partnerDegrees;
            const float elbowRaw = joint == 2 ? bounded : partnerDegrees;
            if (!foldAllowed(shoulderRaw, elbowRaw, runtime)) {
                markLimited();
                return false;
            }
        }
    }
    if (!attached_[joint]) {
        // First command for a joint places it directly; keep it close to the
        // joint's actual mechanical position.
        writeJoint(joint, bounded);
        if (joint == 3) {
            currentGripperDegrees_ = bounded;
            goalGripperDegrees_ = bounded;
        }
    }
    calibrationTarget_[joint] = bounded;
    calibrationPending_[joint] = true;
    return true;
}

void ArmSubsystem::calibrationTick(
    uint32_t elapsedUs, const RuntimeConfig &runtime
) {
    if (!calibrated()) return;
    if (targetLimitedTicks_ > 0) --targetLimitedTicks_;
    if (elapsedUs > config::MaxMotionDtUs) elapsedUs = config::MaxMotionDtUs;
    const float step =
        config::CalibrationDegreesPerSecond * (elapsedUs / 1000000.0F);
    for (uint8_t joint = 0; joint < 4; ++joint) {
        if (!calibrationPending_[joint]) continue;
        const float next =
            moveToward(jointDegrees_[joint], calibrationTarget_[joint], step);
        if (joint == 1 || joint == 2) {
            const uint8_t partner = joint == 1 ? 2 : 1;
            const float shoulderRaw = joint == 1 ? next : jointDegrees_[partner];
            const float elbowRaw = joint == 2 ? next : jointDegrees_[partner];
            if (attached_[partner] &&
                !foldAllowed(shoulderRaw, elbowRaw, runtime)) {
                calibrationPending_[joint] = false;
                markLimited();
                continue;
            }
        }
        writeJoint(joint, next);
        if (joint == 3) {
            currentGripperDegrees_ = next;
            goalGripperDegrees_ = next;
        }
        if (absFloat(next - calibrationTarget_[joint]) < 0.01F)
            calibrationPending_[joint] = false;
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
    targetLimitedTicks_ = 0;
    goal_ = current_;
    goalGripperDegrees_ = currentGripperDegrees_;
}

const ArmTarget &ArmSubsystem::currentTarget() const { return current_; }
const uint8_t *ArmSubsystem::lastCommandedDegrees() const {
    return lastCommandedDegrees_;
}
bool ArmSubsystem::cargoMayBeHeld() const { return cargoMayBeHeld_; }
bool ArmSubsystem::calibrated() const {
    return config::ArmCalibrated || ROBOT_CALIBRATION;
}
bool ArmSubsystem::faulted() const { return faulted_; }
bool ArmSubsystem::targetLimited() const { return targetLimitedTicks_ > 0; }

} // namespace robot
