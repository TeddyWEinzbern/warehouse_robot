#pragma once

#include <Arduino.h>
#include <Servo.h>

#include "domain/RuntimeConfig.h"
#include "domain/RobotTypes.h"

namespace robot {

class ArmSubsystem {
  public:
    ArmSubsystem();
    void begin(const RuntimeConfig &runtime);
    void update(
        const OperatorControlFrame &frame,
        uint32_t elapsedUs,
        const RuntimeConfig &runtime
    );
    void requestPreset(uint16_t pressedButtons, const RuntimeConfig &runtime);
    void requestReach(float reachMm, const RuntimeConfig &runtime);
    void setCalibrationJoint(uint8_t joint, uint8_t degrees);
    void holdLastCommanded();
    void releaseHold();
    void clearFault();
    const ArmTarget &currentTarget() const;
    const uint8_t *lastCommandedDegrees() const;
    bool cargoMayBeHeld() const;
    bool calibrated() const;
    bool faulted() const;

  private:
    Servo servos_[4];
    ArmTarget current_;
    ArmTarget goal_;
    ArmTarget waypoints_[3];
    uint8_t waypointCount_;
    uint8_t waypointIndex_;
    bool cargoMayBeHeld_;
    bool holding_;
    bool faulted_;
    float currentGripperDegrees_;
    float goalGripperDegrees_;
    uint8_t lastCommandedDegrees_[4];
    void planTo(const ArmTarget &target, bool stowing, const RuntimeConfig &runtime);
    bool applyServos(const RuntimeConfig &runtime);
    bool stepTowardGoal(float elapsedSeconds);
    uint8_t mapJoint(
        uint8_t joint,
        float offsetFromCenter,
        const RuntimeConfig &runtime
    ) const;
    static float clampFloat(float value, float low, float high);
};

} // namespace robot
