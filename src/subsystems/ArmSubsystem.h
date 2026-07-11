#pragma once

#include <Arduino.h>
#include <Servo.h>

#include "domain/RobotTypes.h"

namespace robot {

class ArmSubsystem {
  public:
    ArmSubsystem();
    void begin();
    void update(const OperatorControlFrame &frame, uint32_t nowMs);
    void requestPreset(uint16_t pressedButtons);
    void requestReach(float reachMm);
    void setCalibrationJoint(uint8_t joint, uint8_t degrees);
    const ArmTarget &currentTarget() const;
    bool cargoMayBeHeld() const;
    bool calibrated() const;

  private:
    Servo servos_[4];
    ArmTarget current_;
    ArmTarget goal_;
    ArmTarget waypoints_[3];
    uint8_t waypointCount_;
    uint8_t waypointIndex_;
    bool cargoMayBeHeld_;
    float gripperCommandDegrees_;
    uint32_t lastUpdateMs_;
    void planTo(const ArmTarget &target, bool stowing);
    void applyServos();
    bool stepTowardGoal(float elapsedSeconds);
    static float clampFloat(float value, float low, float high);
};

} // namespace robot
