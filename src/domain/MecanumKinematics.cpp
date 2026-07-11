#include "domain/MecanumKinematics.h"

namespace robot {
namespace {
long absLong(long value) { return value < 0 ? -value : value; }
long maxLong(long a, long b) { return a > b ? a : b; }
}

WheelTargets MecanumKinematics::mix(
    int16_t forward, int16_t turn, int16_t strafe
) {
    long fl = static_cast<long>(forward) + strafe + turn;
    long fr = static_cast<long>(forward) - strafe - turn;
    long rl = static_cast<long>(forward) - strafe + turn;
    long rr = static_cast<long>(forward) + strafe - turn;
    const long peak = maxLong(
        maxLong(absLong(fl), absLong(fr)), maxLong(absLong(rl), absLong(rr))
    );
    if (peak > 1000) {
        fl = fl * 1000 / peak;
        fr = fr * 1000 / peak;
        rl = rl * 1000 / peak;
        rr = rr * 1000 / peak;
    }
    WheelTargets result = {
        static_cast<int16_t>(fl), static_cast<int16_t>(fr),
        static_cast<int16_t>(rl), static_cast<int16_t>(rr)
    };
    return result;
}

} // namespace robot

