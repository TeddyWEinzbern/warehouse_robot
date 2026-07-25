#pragma once

#include <Arduino.h>
#include <Servo.h>

#include "domain/ArmKinematics.h"
#include "domain/RuntimeConfig.h"
#include "domain/RobotTypes.h"

namespace robot {

// Drives the four-bar (parallelogram) arm. The elbow servo is grounded on the
// base, so it commands the forearm's absolute pitch; the fold angle between
// the links is what the collision limits constrain. All targets are clamped
// to the reach/height box and to the annulus around the shoulder axis that
// the fold-angle band maps to, before any servo write. Servo writes never
// clamp silently: a target the hardware cannot reach exactly is rejected and
// the commanded state stays where it was.
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
    bool setCalibrationJoint(
        uint8_t joint, uint8_t degrees, const RuntimeConfig &runtime
    );
    void calibrationTick(uint32_t elapsedUs, const RuntimeConfig &runtime);
    void holdLastCommanded();
    void releaseHold();
    void clearFault();
    const ArmTarget &currentTarget() const;
    const uint8_t *lastCommandedDegrees() const;
    bool cargoMayBeHeld() const;
    bool calibrated() const;
    bool faulted() const;
    bool targetLimited() const;
    bool servoTimingActive() const;

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
    uint8_t targetLimitedTicks_;
    bool attached_[4];
    float jointDegrees_[4];
    float calibrationTarget_[4];
    bool calibrationPending_[4];
    float currentGripperDegrees_;
    float goalGripperDegrees_;
    uint8_t lastCommandedDegrees_[4];

    void planTo(const ArmTarget &target, bool stowing, const RuntimeConfig &runtime);
    bool constrainTarget(ArmTarget &target, const RuntimeConfig &runtime) const;
    bool applyServos(const RuntimeConfig &runtime);
    bool stepTowardGoal(float elapsedSeconds, const RuntimeConfig &runtime);
    bool mapJoint(
        uint8_t joint,
        float offsetFromCenter,
        const RuntimeConfig &runtime,
        uint8_t &outDegrees
    ) const;
    void jointOffsetRange(
        uint8_t joint,
        const RuntimeConfig &runtime,
        float &minOffset,
        float &maxOffset
    ) const;
    void writeJoint(uint8_t joint, float degrees);
    bool foldAllowed(
        float shoulderRaw, float elbowRaw, const RuntimeConfig &runtime
    ) const;
    void markLimited();
    static float clampFloat(float value, float low, float high);
};

} // namespace robot
