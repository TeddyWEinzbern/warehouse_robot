#include "subsystems/AssistSubsystem.h"

#include "app/BuildConfig.h"

namespace robot {

AssistSubsystem::AssistSubsystem() : stage_(AssistStage::Idle), startedAtMs_(0) {}
int AssistSubsystem::absolute(int value) { return value < 0 ? -value : value; }
void AssistSubsystem::cancel() { stage_ = AssistStage::Cancelled; }

AssistOutput AssistSubsystem::update(
    const OperatorControlFrame &frame,
    const SensorSnapshot &sensors,
    const ArmTarget &arm,
    bool armCalibrated,
    uint32_t nowMs,
    const RuntimeConfig &runtime
) {
    AssistOutput output = {
        {0, 0, 0, config::AssistDriveLimitPermille, IntentSource::Assist},
        0.0F, stage_, false, false
    };
    if (frame.pressed & StartAssist) { stage_ = AssistStage::Align; startedAtMs_ = nowMs; }
    const int cancelThreshold = config::AssistCancelStickThreshold;
    const bool largeManual = absolute(frame.forward) > cancelThreshold ||
        absolute(frame.turn) > cancelThreshold || absolute(frame.strafe) > cancelThreshold;
    const bool active = stage_ == AssistStage::Align || stage_ == AssistStage::Range;
    if ((frame.pressed & CancelAssist) || largeManual ||
        (active && nowMs - startedAtMs_ > config::AssistTimeoutMs)) {
        stage_ = AssistStage::Cancelled;
        output.stage = stage_;
        return output;
    }
    if (stage_ == AssistStage::Idle || stage_ == AssistStage::Cancelled ||
        stage_ == AssistStage::Complete || !armCalibrated) return output;

    uint8_t direction = 0;
    if (arm.yawDegrees < 45.0F) direction = static_cast<uint8_t>(SensorDirection::Left);
    else if (arm.yawDegrees > 135.0F) direction = static_cast<uint8_t>(SensorDirection::Right);
    const DistancePair &pair = sensors.directions[direction];
    const bool firstFresh = pair.first.valid &&
        nowMs - pair.first.updatedAtMs <= config::SensorStaleMs;
    const bool secondFresh = pair.count < 2 ||
        (pair.second.valid && nowMs - pair.second.updatedAtMs <= config::SensorStaleMs);
    if (!firstFresh || !secondFresh) {
        stage_ = AssistStage::Cancelled;
        output.stage = stage_;
        return output;
    }
    if (pair.count >= 2 && stage_ == AssistStage::Align) {
        const int difference = static_cast<int>(pair.first.millimetres) -
            pair.second.millimetres;
        if (absolute(difference) > static_cast<int>(config::AlignmentToleranceMm)) {
            output.drive.turn = difference > 0
                ? -static_cast<int16_t>(config::AssistDriveLimitPermille)
                : static_cast<int16_t>(config::AssistDriveLimitPermille);
            output.driveActive = true;
            output.stage = stage_;
            return output;
        }
    }
    stage_ = AssistStage::Range;
    const uint16_t distance = pair.count >= 2
        ? static_cast<uint16_t>((pair.first.millimetres + pair.second.millimetres) / 2U)
        : pair.first.millimetres;
    if (distance > runtime.arm.maximumReachMm + runtime.arm.gripperLengthOffsetMm) {
        stage_ = AssistStage::Complete;
    } else if (distance < config::MinAssistDistanceMm) {
        const int16_t away =
            -static_cast<int16_t>(config::AssistDriveLimitPermille);
        if (direction == static_cast<uint8_t>(SensorDirection::Front)) output.drive.forward = away;
        else output.drive.strafe = direction == static_cast<uint8_t>(SensorDirection::Left) ? -away : away;
        output.driveActive = true;
    } else {
        output.requestedReachMm = distance - runtime.arm.gripperLengthOffsetMm;
        output.reachActive = true;
        stage_ = AssistStage::Complete;
    }
    output.stage = stage_;
    return output;
}

} // namespace robot
