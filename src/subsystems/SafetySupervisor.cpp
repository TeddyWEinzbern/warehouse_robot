#include "subsystems/SafetySupervisor.h"

#include "app/BuildConfig.h"

namespace robot {
namespace {
int absolute(int value) { return value < 0 ? -value : value; }
}

SafetySupervisor::SafetySupervisor()
    : state_(RobotState::Boot), faults_(0), neutralSinceMs_(0), neutralTracking_(false),
      linkAlive_(false), immediateStop_(true), clearFaultAccepted_(false) {}

bool SafetySupervisor::neutral(const OperatorControlFrame &frame) {
    return frame.valid && absolute(frame.forward) <= 30 && absolute(frame.turn) <= 30 &&
           absolute(frame.strafe) <= 30 && absolute(frame.armYaw) <= 30 &&
           absolute(frame.armReach) <= 30 && absolute(frame.armHeight) <= 30 &&
           frame.gripper == 0 && (frame.controlFlags & EStopAsserted) == 0;
}

void SafetySupervisor::transition(RobotState next) {
    if (state_ == next) return;
    if (state_ == RobotState::Armed || next == RobotState::EStop || next == RobotState::Fault)
        immediateStop_ = true;
    state_ = next;
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
        transition(RobotState::Fault);
    }
    if ((frame.controlFlags & EStopAsserted) != 0) transition(RobotState::EStop);

    if (state_ == RobotState::Boot && platformInitialized && drive.initialized)
        transition(RobotState::Disarmed);

    if ((requests.flags & RequestDisarm) != 0 && state_ == RobotState::Armed)
        transition(RobotState::Disarmed);
    if (state_ == RobotState::Armed && !linkAlive_) transition(RobotState::Disarmed);

    if (state_ == RobotState::Disarmed && (requests.flags & RequestArm) != 0 &&
        profileCanArm && drive.feedbackHealthy && linkAlive_ && neutralQualified)
        transition(RobotState::Armed);

    if (state_ == RobotState::EStop && (requests.flags & RequestClearEStop) != 0 &&
        (frame.controlFlags & EStopAsserted) == 0 && neutralQualified) {
        transition(faults_ == 0 ? RobotState::Disarmed : RobotState::Fault);
    }

    if (state_ == RobotState::Fault && (requests.flags & RequestClearFault) != 0 &&
        drive.feedbackHealthy && neutralQualified) {
        faults_ = 0;
        clearFaultAccepted_ = true;
        transition(RobotState::Disarmed);
    }
}

DriveIntent SafetySupervisor::arbitrate(
    const OperatorControlFrame &frame, const AssistOutput &assist,
    bool cargoMayBeHeld, const RuntimeConfig &runtime
) const {
    if (state_ != RobotState::Armed || !linkAlive_)
        return {0, 0, 0, 0, IntentSource::Safety};
    const uint16_t limit = cargoMayBeHeld
        ? runtime.cargoDriveLimitPermille : runtime.normalDriveLimitPermille;
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
    transition(RobotState::Fault);
}
bool SafetySupervisor::takeImmediateStop() { const bool value = immediateStop_; immediateStop_ = false; return value; }
bool SafetySupervisor::takeClearFaultAccepted() { const bool value = clearFaultAccepted_; clearFaultAccepted_ = false; return value; }
RobotState SafetySupervisor::state() const { return state_; }
bool SafetySupervisor::armed() const { return state_ == RobotState::Armed; }
bool SafetySupervisor::linkAlive() const { return linkAlive_; }
bool SafetySupervisor::emergencyStopped() const { return state_ == RobotState::EStop; }
uint16_t SafetySupervisor::faults() const { return faults_; }

} // namespace robot
