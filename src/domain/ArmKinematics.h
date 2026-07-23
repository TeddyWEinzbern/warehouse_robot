#pragma once

namespace robot {

struct JointSolution {
    float shoulderDegrees;
    float elbowDegrees;
    bool reachable;
};

struct PlanarLimits {
    float minRadialMm;
    float maxRadialMm;
};

class ArmKinematics {
  public:
    // Elbow-up solution matching the physical linkage: shoulderDegrees is the
    // upper-arm angle from horizontal, elbowDegrees the fold angle between the
    // upper arm and forearm (0 = straight). Forearm absolute angle is
    // shoulderDegrees - elbowDegrees.
    static JointSolution solvePlanar(
        float reachMm, float heightMm, float firstLinkMm, float secondLinkMm
    );
    // Wrist-to-shoulder distance band equivalent to a fold-angle band. The
    // fold angle depends only on this distance, so the four-bar collision
    // limit is an annulus around the shoulder axis.
    static PlanarLimits radialLimits(
        float firstLinkMm, float secondLinkMm,
        float foldMinDegrees, float foldMaxDegrees
    );
    // Radially projects (reachMm, heightMm relative to the shoulder axis)
    // onto the annulus. Returns true when the point had to move.
    static bool constrainPlanar(
        float &reachMm, float &heightMm, const PlanarLimits &limits
    );
};

} // namespace robot
