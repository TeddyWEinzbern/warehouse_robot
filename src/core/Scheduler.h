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

// Fixed-rate deadline without diagnostics. Use this for non-safety-critical
// housekeeping tasks so each task does not carry counters that are never read.
class PeriodicDeadline {
  public:
    PeriodicDeadline() : periodUs_(0), deadlineUs_(0), started_(false) {}
    void start(
        uint32_t nowUs, uint32_t periodUs, uint32_t phaseUs = 0
    ) {
        periodUs_ = periodUs;
        deadlineUs_ = nowUs + phaseUs;
        started_ = true;
    }
    bool due(uint32_t nowUs) {
        if (!started_ ||
            static_cast<int32_t>(nowUs - deadlineUs_) < 0)
            return false;
        const uint32_t lateness = nowUs - deadlineUs_;
        const uint32_t missed = periodUs_ == 0
            ? 0 : lateness / periodUs_;
        deadlineUs_ += (missed + 1UL) * periodUs_;
        return true;
    }
    uint32_t untilDeadlineUs(uint32_t nowUs) const {
        if (static_cast<int32_t>(deadlineUs_ - nowUs) <= 0) return 0;
        return deadlineUs_ - nowUs;
    }

  private:
    uint32_t periodUs_;
    uint32_t deadlineUs_;
    bool started_;
};

class SchedulerHealth {
  public:
    SchedulerHealth();
    void begin(uint32_t nowUs);
    bool observeLoop(uint32_t nowUs, bool armed);

  private:
    uint32_t lastLoopUs_;
};

} // namespace robot
