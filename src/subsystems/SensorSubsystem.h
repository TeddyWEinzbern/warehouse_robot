#pragma once

#include <Arduino.h>

#include "domain/RobotTypes.h"

namespace robot {

class SensorSubsystem {
  public:
    SensorSubsystem();
    void begin();
    void update(uint32_t nowMs, uint32_t nowUs);
    const SensorSnapshot &snapshot() const;

  private:
    enum class PingState : uint8_t { Start, TriggerHigh, WaitRise, WaitFall, Gap };
    SensorSnapshot snapshot_;
    PingState state_;
    uint8_t sensorIndex_;
    uint32_t stateStartedUs_;
    uint32_t echoStartedUs_;
    void storeDistance(uint16_t millimetres, uint32_t nowMs, bool valid);
};

} // namespace robot

