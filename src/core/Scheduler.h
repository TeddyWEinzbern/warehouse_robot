#pragma once

#include <stdint.h>

#include "domain/RobotTypes.h"

namespace robot {

class PeriodicTask {
  public:
    PeriodicTask();
    void start(uint32_t nowUs, uint32_t periodUs, uint32_t phaseUs = 0);
    bool due(uint32_t nowUs);
    uint32_t elapsedUs(uint32_t nowUs, uint32_t maximumUs);
    uint32_t untilDeadlineUs(uint32_t nowUs) const;
    uint32_t lastLatenessUs() const;
    const SchedulerTaskStats &stats() const;
    SchedulerTaskStats &stats();

  private:
    uint32_t periodUs_;
    uint32_t deadlineUs_;
    uint32_t lastRunUs_;
    uint32_t lastLatenessUs_;
    SchedulerTaskStats stats_;
    bool started_;
};

class SchedulerHealth {
  public:
    SchedulerHealth();
    void begin(uint32_t nowUs);
    bool observeLoop(uint32_t nowUs, bool armed);
    uint32_t maxLoopGapUs() const;

  private:
    uint32_t lastLoopUs_;
    uint32_t maxLoopGapUs_;
};

} // namespace robot
