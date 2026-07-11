#pragma once

namespace robot {

struct JointSolution {
    float shoulderDegrees;
    float elbowDegrees;
    bool reachable;
};

class ArmKinematics {
  public:
    static JointSolution solvePlanar(
        float reachMm, float heightMm, float firstLinkMm, float secondLinkMm
    );
};

} // namespace robot

