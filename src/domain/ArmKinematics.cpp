#include "domain/ArmKinematics.h"

#include <math.h>

namespace robot {
namespace {
constexpr float RadiansToDegrees = 57.2957795F;
constexpr float DegreesToRadians = 0.0174532925F;
}

JointSolution ArmKinematics::solvePlanar(
    float reachMm, float heightMm, float firstLinkMm, float secondLinkMm
) {
    const float distanceSquared = reachMm * reachMm + heightMm * heightMm;
    const float cosineElbow =
        (distanceSquared - firstLinkMm * firstLinkMm -
         secondLinkMm * secondLinkMm) /
        (2.0F * firstLinkMm * secondLinkMm);
    if (cosineElbow < -1.0F || cosineElbow > 1.0F) {
        return {0.0F, 0.0F, false};
    }
    const float elbow = acosf(cosineElbow);
    const float shoulder = atan2f(heightMm, reachMm) +
                           atan2f(
                               secondLinkMm * sinf(elbow),
                               firstLinkMm + secondLinkMm * cosf(elbow)
                           );
    return {
        shoulder * RadiansToDegrees,
        elbow * RadiansToDegrees,
        true,
    };
}

PlanarLimits ArmKinematics::radialLimits(
    float firstLinkMm, float secondLinkMm,
    float foldMinDegrees, float foldMaxDegrees
) {
    const float crossTerm = 2.0F * firstLinkMm * secondLinkMm;
    const float sumSquares =
        firstLinkMm * firstLinkMm + secondLinkMm * secondLinkMm;
    const float minSquared =
        sumSquares + crossTerm * cosf(foldMaxDegrees * DegreesToRadians);
    const float maxSquared =
        sumSquares + crossTerm * cosf(foldMinDegrees * DegreesToRadians);
    return {
        minSquared > 0.0F ? sqrtf(minSquared) : 0.0F,
        maxSquared > 0.0F ? sqrtf(maxSquared) : 0.0F,
    };
}

bool ArmKinematics::constrainPlanar(
    float &reachMm, float &heightMm, const PlanarLimits &limits
) {
    const float distance = sqrtf(reachMm * reachMm + heightMm * heightMm);
    if (distance < 1.0F) {
        reachMm = limits.minRadialMm;
        heightMm = 0.0F;
        return true;
    }
    float scale = 1.0F;
    if (distance < limits.minRadialMm) scale = limits.minRadialMm / distance;
    else if (distance > limits.maxRadialMm) scale = limits.maxRadialMm / distance;
    else return false;
    reachMm *= scale;
    heightMm *= scale;
    return true;
}

} // namespace robot
