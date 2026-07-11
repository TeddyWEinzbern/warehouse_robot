#include "subsystems/SensorSubsystem.h"

#include "app/PinProfile.h"

namespace robot {

SensorSubsystem::SensorSubsystem()
    : snapshot_({}), state_(PingState::Start), sensorIndex_(0),
      stateStartedUs_(0), echoStartedUs_(0) {}

void SensorSubsystem::begin() {
    pinMode(pins::SonarTrigger, OUTPUT);
    digitalWrite(pins::SonarTrigger, LOW);
    for (uint8_t index = 0; index < pins::SonarCount; ++index)
        pinMode(pins::SonarEcho[index], INPUT);
}

void SensorSubsystem::storeDistance(uint16_t millimetres, uint32_t nowMs, bool valid) {
    const uint8_t direction = pins::SonarDirection[sensorIndex_];
    const uint8_t slot = pins::SonarSlot[sensorIndex_];
    DistancePair &pair = snapshot_.directions[direction];
    if (slot == 0) pair.firstMm = millimetres;
    else pair.secondMm = millimetres;
    if (pair.count < slot + 1) pair.count = slot + 1;
    pair.updatedAtMs = nowMs;
    pair.valid = valid && pair.firstMm > 0 && (pair.count < 2 || pair.secondMm > 0);
}

void SensorSubsystem::update(uint32_t nowMs, uint32_t nowUs) {
    const uint8_t echoPin = pins::SonarEcho[sensorIndex_];
    switch (state_) {
        case PingState::Start:
            digitalWrite(pins::SonarTrigger, HIGH);
            stateStartedUs_ = nowUs;
            state_ = PingState::TriggerHigh;
            break;
        case PingState::TriggerHigh:
            if (nowUs - stateStartedUs_ >= 10UL) {
                digitalWrite(pins::SonarTrigger, LOW);
                stateStartedUs_ = nowUs;
                state_ = PingState::WaitRise;
            }
            break;
        case PingState::WaitRise:
            if (digitalRead(echoPin) == HIGH) {
                echoStartedUs_ = nowUs;
                state_ = PingState::WaitFall;
            } else if (nowUs - stateStartedUs_ > 25000UL) {
                storeDistance(0, nowMs, false);
                stateStartedUs_ = nowUs;
                state_ = PingState::Gap;
            }
            break;
        case PingState::WaitFall:
            if (digitalRead(echoPin) == LOW) {
                const uint32_t duration = nowUs - echoStartedUs_;
                storeDistance(static_cast<uint16_t>(duration * 10UL / 58UL), nowMs, true);
                stateStartedUs_ = nowUs;
                state_ = PingState::Gap;
            } else if (nowUs - echoStartedUs_ > 25000UL) {
                storeDistance(0, nowMs, false);
                stateStartedUs_ = nowUs;
                state_ = PingState::Gap;
            }
            break;
        case PingState::Gap:
            if (nowUs - stateStartedUs_ >= 30000UL) {
                sensorIndex_ = static_cast<uint8_t>((sensorIndex_ + 1) % pins::SonarCount);
                state_ = PingState::Start;
            }
            break;
    }
}

const SensorSnapshot &SensorSubsystem::snapshot() const { return snapshot_; }

} // namespace robot

