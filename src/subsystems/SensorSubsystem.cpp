#include "subsystems/SensorSubsystem.h"

#include "app/PinProfile.h"

namespace robot {

SensorSubsystem::SensorSubsystem()
    : snapshot_{}, state_(PingState::Idle), echoState_{}, activeGroup_(0),
      nextGroup_(0), stateStartedUs_(0), echoStartedUs_{0, 0, 0, 0, 0, 0},
      offsetsMm_{0, 0, 0, 0, 0, 0} {}

void SensorSubsystem::begin() {
    for (uint8_t group = 0; group < pins::SonarGroupCount; ++group) {
        pinMode(pins::SonarTrigger[group], OUTPUT);
        digitalWrite(pins::SonarTrigger[group], LOW);
    }
    for (uint8_t sensor = 0; sensor < pins::SonarCount; ++sensor)
        pinMode(pins::SonarEcho[sensor], INPUT);
}

void SensorSubsystem::storeDistance(
    uint8_t sensor, uint16_t millimetres, uint32_t nowMs, bool valid
) {
    const uint8_t direction = pins::SonarDirection[sensor];
    const uint8_t slot = pins::SonarSlot[sensor];
    DistancePair &pair = snapshot_.directions[direction];
    DistanceReading &reading = slot == 0 ? pair.first : pair.second;
    int32_t adjusted = static_cast<int32_t>(millimetres) + offsetsMm_[sensor];
    if (adjusted < 1 || adjusted > 65535) valid = false;
    reading.millimetres = valid ? static_cast<uint16_t>(adjusted) : 0;
    reading.updatedAtMs = nowMs;
    reading.valid = valid;
    if (pair.count < slot + 1) pair.count = slot + 1;
}

bool SensorSubsystem::startNextGroup(
    uint32_t, uint32_t nowUs, const RuntimeConfig &runtime
) {
    if (state_ != PingState::Idle) return false;
    activeGroup_ = nextGroup_;
    nextGroup_ = static_cast<uint8_t>((nextGroup_ + 1U) % pins::SonarGroupCount);
    for (uint8_t sensor = 0; sensor < pins::SonarCount; ++sensor) {
        offsetsMm_[sensor] = runtime.sensorOffsetMm[sensor];
        echoState_[sensor] = pins::SonarGroup[sensor] == activeGroup_
            ? EchoState::WaitRise : EchoState::Inactive;
    }
    digitalWrite(pins::SonarTrigger[activeGroup_], HIGH);
    stateStartedUs_ = nowUs;
    state_ = PingState::TriggerHigh;
    return true;
}

bool SensorSubsystem::groupComplete() const {
    for (uint8_t sensor = 0; sensor < pins::SonarCount; ++sensor) {
        if (pins::SonarGroup[sensor] == activeGroup_ &&
            echoState_[sensor] != EchoState::Complete) return false;
    }
    return true;
}

void SensorSubsystem::poll(uint32_t nowMs, uint32_t nowUs) {
    switch (state_) {
        case PingState::Idle: break;
        case PingState::TriggerHigh:
            if (nowUs - stateStartedUs_ >= 10UL) {
                digitalWrite(pins::SonarTrigger[activeGroup_], LOW);
                stateStartedUs_ = nowUs;
                state_ = PingState::Capture;
            }
            break;
        case PingState::Capture:
            for (uint8_t sensor = 0; sensor < pins::SonarCount; ++sensor) {
                if (pins::SonarGroup[sensor] != activeGroup_) continue;
                if (echoState_[sensor] == EchoState::WaitRise &&
                    digitalRead(pins::SonarEcho[sensor]) == HIGH) {
                    echoStartedUs_[sensor] = nowUs;
                    echoState_[sensor] = EchoState::WaitFall;
                } else if (echoState_[sensor] == EchoState::WaitFall &&
                           digitalRead(pins::SonarEcho[sensor]) == LOW) {
                    const uint32_t duration = nowUs - echoStartedUs_[sensor];
                    storeDistance(
                        sensor, static_cast<uint16_t>(duration * 10UL / 58UL), nowMs, true
                    );
                    echoState_[sensor] = EchoState::Complete;
                }
                if (echoState_[sensor] != EchoState::Complete &&
                    nowUs - stateStartedUs_ > 25000UL) {
                    storeDistance(sensor, 0, nowMs, false);
                    echoState_[sensor] = EchoState::Complete;
                }
            }
            if (groupComplete()) state_ = PingState::Idle;
            break;
    }
}

bool SensorSubsystem::capturing() const { return state_ != PingState::Idle; }
const SensorSnapshot &SensorSubsystem::snapshot() const { return snapshot_; }

} // namespace robot
