#pragma once

#include "domain/RobotTypes.h"

namespace robot {

class SafetySupervisor {
  public:
    SafetySupervisor();
    void update(
        const OperatorControlFrame &frame,
        const ControlRequests &requests,
        const DriveHealth &drive,
        bool platformInitialized,
        bool profileCanArm,
        uint32_t nowMs
    );
    DriveIntent arbitrate(
        const OperatorControlFrame &frame,
        const AssistOutput &assist,
        bool cargoMayBeHeld
    ) const;
    void latchFault(uint16_t fault);
    bool takeImmediateStop();
    bool takeClearFaultAccepted();
    RobotState state() const;
    bool armed() const;
    bool linkAlive() const;
    bool emergencyStopped() const;
    uint16_t faults() const;

  private:
    RobotState state_;
    uint16_t faults_;
    uint32_t neutralSinceMs_;
    bool neutralTracking_;
    bool linkAlive_;
    bool immediateStop_;
    bool clearFaultAccepted_;
    static bool neutral(const OperatorControlFrame &frame);
    void transition(RobotState next);
};

} // namespace robot
