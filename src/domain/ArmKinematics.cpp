#include "domain/ArmKinematics.h"

#include <math.h>

namespace robot {

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
    const float shoulder = atan2f(heightMm, reachMm) -
                           atan2f(
                               secondLinkMm * sinf(elbow),
                               firstLinkMm + secondLinkMm * cosf(elbow)
                           );
    const float radiansToDegrees = 57.2957795F;
    return {
        shoulder * radiansToDegrees,
        elbow * radiansToDegrees,
        true,
    };
}

} // namespace robot

