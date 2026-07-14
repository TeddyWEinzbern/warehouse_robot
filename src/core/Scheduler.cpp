#include "core/Scheduler.h"

#include "app/BuildConfig.h"

namespace robot {
namespace {
void saturatingAdd(uint16_t &value, uint32_t amount) {
    const uint32_t sum = static_cast<uint32_t>(value) + amount;
    value = static_cast<uint16_t>(sum > 65535UL ? 65535UL : sum);
}
}

PeriodicTask::PeriodicTask()
    : periodUs_(0), deadlineUs_(0), lastRunUs_(0), lastLatenessUs_(0),
      stats_({0, 0, 0}), started_(false) {}

void PeriodicTask::start(uint32_t nowUs, uint32_t periodUs, uint32_t phaseUs) {
    periodUs_ = periodUs;
    deadlineUs_ = nowUs + phaseUs;
    lastRunUs_ = nowUs;
    lastLatenessUs_ = 0;
    stats_ = {0, 0, 0};
    started_ = true;
}

bool PeriodicTask::due(uint32_t nowUs) {
    if (!started_ || static_cast<int32_t>(nowUs - deadlineUs_) < 0) return false;
    const uint32_t lateness = nowUs - deadlineUs_;
    lastLatenessUs_ = lateness;
    if (lateness > stats_.maxLatenessUs) stats_.maxLatenessUs = lateness;
    const uint32_t missedPeriods = periodUs_ == 0 ? 0 : lateness / periodUs_;
    if (missedPeriods > 0) {
        saturatingAdd(stats_.missed, missedPeriods);
        saturatingAdd(stats_.consecutiveMisses, missedPeriods);
    } else {
        stats_.consecutiveMisses = 0;
    }
    deadlineUs_ += (missedPeriods + 1UL) * periodUs_;
    return true;
}

uint32_t PeriodicTask::elapsedUs(uint32_t nowUs, uint32_t maximumUs) {
    uint32_t elapsed = nowUs - lastRunUs_;
    lastRunUs_ = nowUs;
    if (elapsed > maximumUs) elapsed = maximumUs;
    return elapsed;
}

uint32_t PeriodicTask::untilDeadlineUs(uint32_t nowUs) const {
    if (static_cast<int32_t>(deadlineUs_ - nowUs) <= 0) return 0;
    return deadlineUs_ - nowUs;
}
uint32_t PeriodicTask::lastLatenessUs() const { return lastLatenessUs_; }

const SchedulerTaskStats &PeriodicTask::stats() const { return stats_; }
SchedulerTaskStats &PeriodicTask::stats() { return stats_; }

SchedulerHealth::SchedulerHealth() : lastLoopUs_(0), maxLoopGapUs_(0) {}
void SchedulerHealth::begin(uint32_t nowUs) { lastLoopUs_ = nowUs; maxLoopGapUs_ = 0; }
bool SchedulerHealth::observeLoop(uint32_t nowUs, bool armed) {
    const uint32_t gap = nowUs - lastLoopUs_;
    lastLoopUs_ = nowUs;
    if (gap > maxLoopGapUs_) maxLoopGapUs_ = gap;
    return armed && gap > config::MaxLoopGapUs;
}
uint32_t SchedulerHealth::maxLoopGapUs() const { return maxLoopGapUs_; }

} // namespace robot
