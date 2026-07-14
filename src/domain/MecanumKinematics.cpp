#include "domain/MecanumKinematics.h"

namespace robot {
namespace {
int32_t absolute32(int32_t value) { return value < 0 ? -value : value; }
int32_t maximum32(int32_t a, int32_t b) { return a > b ? a : b; }
}

WheelTargets MecanumKinematics::mix(
    const ChassisVelocity &velocity,
    uint16_t wheelTrackMm,
    uint16_t wheelbaseMm,
    uint16_t maximumWheelMmS
) {
    const int32_t leverArmMm = (static_cast<int32_t>(wheelTrackMm) + wheelbaseMm) / 2L;
    const int32_t rotational = static_cast<int32_t>(velocity.yawMradS) * leverArmMm / 1000L;
    int32_t fl = static_cast<int32_t>(velocity.longitudinalMmS) + velocity.lateralMmS + rotational;
    int32_t fr = static_cast<int32_t>(velocity.longitudinalMmS) - velocity.lateralMmS - rotational;
    int32_t rl = static_cast<int32_t>(velocity.longitudinalMmS) - velocity.lateralMmS + rotational;
    int32_t rr = static_cast<int32_t>(velocity.longitudinalMmS) + velocity.lateralMmS - rotational;
    const int32_t peak = maximum32(
        maximum32(absolute32(fl), absolute32(fr)),
        maximum32(absolute32(rl), absolute32(rr))
    );
    if (peak > maximumWheelMmS && peak > 0) {
        fl = fl * maximumWheelMmS / peak;
        fr = fr * maximumWheelMmS / peak;
        rl = rl * maximumWheelMmS / peak;
        rr = rr * maximumWheelMmS / peak;
    }
    return {
        static_cast<int16_t>(fl), static_cast<int16_t>(fr),
        static_cast<int16_t>(rl), static_cast<int16_t>(rr)
    };
}

} // namespace robot
