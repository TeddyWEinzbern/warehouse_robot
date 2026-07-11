#pragma once

#include "drivers/DriveBackend.h"

namespace robot {

class ChassisSubsystem {
  public:
    explicit ChassisSubsystem(DriveBackend &backend);
    void begin();
    void update(const DriveIntent &intent, uint32_t nowMs);
    void updateDirect(const WheelTargets &targets, uint16_t limit, uint32_t nowMs);
    void stop();
    DriveBackend &backend();

  private:
    DriveBackend &backend_;
    WheelTargets last_;
    uint32_t lastUpdateMs_;
    bool stopped_;
    static int16_t clampAxis(long value, uint16_t limit);
    static int16_t approach(int16_t current, int16_t target, int16_t step);
};

} // namespace robot
