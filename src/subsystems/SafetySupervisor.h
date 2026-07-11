#pragma once

#include "domain/RobotTypes.h"

namespace robot {

class SafetySupervisor {
  public:
    SafetySupervisor();
    DriveIntent arbitrate(
        const OperatorControlFrame &frame,
        const AssistOutput &assist,
        bool cargoMayBeHeld,
        uint32_t nowMs
    );
    bool linkAlive() const;
    bool emergencyStopped() const;

  private:
    bool linkAlive_;
    bool emergencyStopped_;
};

} // namespace robot

