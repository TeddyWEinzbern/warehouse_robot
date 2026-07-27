#include "subsystems/SafetySupervisor.h"

#include "app/BuildConfig.h"

namespace robot {
namespace {
int absolute(int value) { return value < 0 ? -value : value; }
}

SafetySupervisor::SafetySupervisor()
    : state_(RobotState::Disarmed), faults_(0), neutralSinceMs_(0),
      neutralTracking_(false), linkAlive_(false), readyToArm_(false),
      immediateStop_(true), clearFaultAccepted_(false) {}

bool SafetySupervisor::neutral(const OperatorControlFrame &frame) {
    return frame.valid && absolute(frame.forward) <= 30 && absolute(frame.turn) <= 30 &&
           absolute(frame.strafe) <= 30 && absolute(frame.armYaw) <= 30 &&
           absolute(frame.armReach) <= 30 && absolute(frame.armHeight) <= 30 &&
           frame.gripper == 0 && frame.buttons == 0;
}

void SafetySupervisor::transition(RobotState next) {
    if (state_ == next) return;
    if (state_ == RobotState::Armed || next == RobotState::Fault)
        immediateStop_ = true;
    state_ = next;
    if (next != RobotState::Disarmed) readyToArm_ = false;
}

void SafetySupervisor::update(
    const OperatorControlFrame &frame, const ControlRequests &requests,
    const DriveHealth &drive, bool platformInitialized, bool profileCanArm,
    uint32_t nowMs
) {
    clearFaultAccepted_ = false;
    linkAlive_ = frame.valid && nowMs - frame.receivedAtMs <= config::CommandTimeoutMs;
    if (neutral(frame) && linkAlive_) {
        if (!neutralTracking_) { neutralTracking_ = true; neutralSinceMs_ = nowMs; }
    } else neutralTracking_ = false;
    const bool neutralQualified = neutralTracking_ &&
        nowMs - neutralSinceMs_ >= config::NeutralQualificationMs;

    if (drive.faults != 0) {
        faults_ |= drive.faults;
        if (!config::FaultStateUnsafe)
            transition(RobotState::Fault);
    }
    const bool disarmRequested = (requests.flags & RequestDisarm) != 0;

    if (disarmRequested) {
        // DISARM is dominant over every request that could relax a safety
        // state. It also requests an immediate stop while already DISARMED,
        // which cancels bounded calibration motion.
        immediateStop_ = true;
        if (state_ == RobotState::Armed)
            transition(RobotState::Disarmed);
    }
    if (state_ == RobotState::Armed && !linkAlive_) transition(RobotState::Disarmed);

    const bool clearableFaultState =
        state_ == RobotState::Fault ||
        (config::FaultStateUnsafe &&
         state_ == RobotState::Disarmed && faults_ != 0);
    const bool clearFaultRequested =
        (requests.flags & RequestClearFault) != 0;
    const bool feedbackRecovered =
        drive.feedbackHealthy &&
        (drive.warnings & WarningEncoderTimeoutIgnored) == 0;
    if (!disarmRequested &&
        clearableFaultState &&
        clearFaultRequested &&
        feedbackRecovered && neutralQualified) {
        faults_ = 0;
        clearFaultAccepted_ = true;
        transition(RobotState::Disarmed);
    }

    readyToArm_ =
        state_ == RobotState::Disarmed &&
        platformInitialized &&
        profileCanArm &&
        drive.initialized &&
        drive.feedbackReady &&
        drive.feedbackHealthy &&
        faults_ == 0 &&
        linkAlive_ &&
        neutralQualified;

    // Clearing a fault always ends in DISARMED, even if ARM arrived in the
    // same batch. DISARM likewise dominates both state-relaxing requests.
    if (!disarmRequested && !clearFaultRequested &&
        (requests.flags & RequestArm) != 0 &&
        readyToArm_)
        transition(RobotState::Armed);
}

DriveIntent SafetySupervisor::arbitrate(
    const OperatorControlFrame &frame, const AssistOutput &assist,
    bool cargoMayBeHeld
) const {
    if (state_ != RobotState::Armed || !linkAlive_)
        return {0, 0, 0, 0, IntentSource::Safety};
    const uint16_t limit = cargoMayBeHeld
        ? config::CargoDriveLimitPermille : config::NormalDriveLimitPermille;
    if (assist.driveActive) {
        DriveIntent result = assist.drive;
        const int16_t blend = static_cast<int16_t>(config::AssistManualBlendThreshold);
        if (frame.forward < blend && frame.forward > -blend) result.forward += frame.forward;
        if (frame.turn < blend && frame.turn > -blend) result.turn += frame.turn;
        if (frame.strafe < blend && frame.strafe > -blend) result.strafe += frame.strafe;
        if (result.maxMagnitudePermille > limit) result.maxMagnitudePermille = limit;
        return result;
    }
    return {frame.forward, frame.turn, frame.strafe, limit, IntentSource::Operator};
}

void SafetySupervisor::latchFault(uint16_t fault) {
    faults_ |= fault;
    readyToArm_ = false;
    if (!config::FaultStateUnsafe)
        transition(RobotState::Fault);
}
bool SafetySupervisor::takeImmediateStop() { const bool value = immediateStop_; immediateStop_ = false; return value; }
bool SafetySupervisor::takeClearFaultAccepted() { const bool value = clearFaultAccepted_; clearFaultAccepted_ = false; return value; }
RobotState SafetySupervisor::state() const { return state_; }
bool SafetySupervisor::armed() const { return state_ == RobotState::Armed; }
bool SafetySupervisor::linkAlive() const { return linkAlive_; }
bool SafetySupervisor::readyToArm() const { return readyToArm_; }
uint16_t SafetySupervisor::faults() const { return faults_; }

} // namespace robot
