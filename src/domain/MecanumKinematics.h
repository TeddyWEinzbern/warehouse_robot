#pragma once

#include "domain/RobotTypes.h"

namespace robot {

class MecanumKinematics {
  public:
    static WheelTargets mix(int16_t forward, int16_t turn, int16_t strafe);
};

} // namespace robot

