#pragma once

#include "domain/RobotTypes.h"

namespace robot {

class AssistSubsystem {
  public:
    AssistSubsystem();
    AssistOutput update(
        const OperatorControlFrame &frame,
        const SensorSnapshot &sensors,
        const ArmTarget &arm,
        bool armCalibrated,
        uint32_t nowMs
    );

  private:
    AssistStage stage_;
    uint32_t startedAtMs_;
    static int absolute(int value);
};

} // namespace robot

