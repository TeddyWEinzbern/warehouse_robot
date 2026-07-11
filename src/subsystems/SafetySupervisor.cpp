#include "subsystems/SafetySupervisor.h"

#include "app/BuildConfig.h"

namespace robot {

SafetySupervisor::SafetySupervisor() : linkAlive_(false), emergencyStopped_(false) {}

DriveIntent SafetySupervisor::arbitrate(
    const OperatorControlFrame &frame,
    const AssistOutput &assist,
    bool cargoMayBeHeld,
    uint32_t nowMs
) {
    if (frame.pressed & EmergencyStop) emergencyStopped_ = true;
    if (frame.pressed & ClearEmergencyStop) emergencyStopped_ = false;
    linkAlive_ = frame.valid && nowMs - frame.receivedAtMs <= config::CommandTimeoutMs;
    const uint16_t limit = cargoMayBeHeld ? config::CargoDriveLimit : config::NormalDriveLimit;
    if (!linkAlive_ || emergencyStopped_) return {0, 0, 0, 0, IntentSource::Safety};
    if (assist.driveActive) {
        DriveIntent result = assist.drive;
        const int16_t blend = static_cast<int16_t>(config::AssistManualBlendThreshold);
        if (frame.forward < blend && frame.forward > -blend)
            result.forward += frame.forward;
        if (frame.turn < blend && frame.turn > -blend)
            result.turn += frame.turn;
        if (frame.strafe < blend && frame.strafe > -blend)
            result.strafe += frame.strafe;
        result.maxMagnitude = result.maxMagnitude < limit ? result.maxMagnitude : limit;
        return result;
    }
    return {frame.forward, frame.turn, frame.strafe, limit, IntentSource::Operator};
}

bool SafetySupervisor::linkAlive() const { return linkAlive_; }
bool SafetySupervisor::emergencyStopped() const { return emergencyStopped_; }

} // namespace robot
