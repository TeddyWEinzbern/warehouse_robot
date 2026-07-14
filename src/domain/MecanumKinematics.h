#pragma once

#include "domain/RobotTypes.h"

namespace robot {

class MecanumKinematics {
  public:
    static WheelTargets mix(
        const ChassisVelocity &velocity,
        uint16_t wheelTrackMm,
        uint16_t wheelbaseMm,
        uint16_t maximumWheelMmS
    );
};

} // namespace robot
