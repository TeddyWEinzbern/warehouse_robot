#pragma once

#include <Arduino.h>

#include "domain/RobotTypes.h"

namespace robot {

class SensorSubsystem {
  public:
    SensorSubsystem();
    void begin();
    void poll(uint32_t nowMs, uint32_t nowUs);
    bool startNextGroup(uint32_t nowMs, uint32_t nowUs);
    bool capturing() const;
    const SensorSnapshot &snapshot() const;

  private:
    enum class PingState : uint8_t { Idle, TriggerHigh, Capture };
    enum class EchoState : uint8_t { Inactive, WaitRise, WaitFall, Complete };
    SensorSnapshot snapshot_;
    PingState state_;
    EchoState echoState_[6];
    uint8_t activeGroup_;
    uint8_t nextGroup_;
    uint32_t stateStartedUs_;
    uint32_t echoStartedUs_[6];
    void storeDistance(uint8_t sensor, uint16_t millimetres, uint32_t nowMs, bool valid);
    bool groupComplete() const;
};

} // namespace robot
