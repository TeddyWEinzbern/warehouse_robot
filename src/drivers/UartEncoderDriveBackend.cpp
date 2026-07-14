#include "drivers/UartEncoderDriveBackend.h"

#include "app/BuildConfig.h"

#include <string.h>

namespace robot {
namespace {
int32_t absolute32(int32_t value) { return value < 0 ? -value : value; }
void saturatingIncrement(uint16_t &value) { if (value != 65535U) ++value; }
bool elapsed(uint32_t now, uint32_t since, uint32_t duration) { return now - since >= duration; }
}

UartEncoderDriveBackend::UartEncoderDriveBackend(HardwareSerial &serial)
    : serial_(serial), feedback_({}), targets_({0, 0, 0, 0}),
      initStage_(InitStage::Settling), outstanding_(QueryType::None),
      startedAtMs_(0), querySentAtMs_(0), lastZeroAtMs_(0),
      lastMotorCommandAtMs_(0), lastEncoderCompletedAtMs_(0),
      lastEncoderDeadlineAtMs_(0),
      badSignSinceMs_{0, 0, 0, 0}, stallSinceMs_{0, 0, 0, 0},
      mismatchSinceMs_{0, 0, 0, 0}, motionStartedAtMs_{0, 0, 0, 0},
      previousTargetMmS_{0, 0, 0, 0}, previousMeasuredMmS_{0, 0, 0, 0},
      faults_(0), warnings_(0), queryTimeouts_(0),
      consecutiveValid_(0), consecutiveMalformed_(0), implausibleSamples_(0),
      unchangedTotalFrames_{0, 0, 0, 0},
      parser_(), armed_(false), pendingZero_(false),
      pendingMotor_(false), encoderDue_(false), totalDue_(false), batteryDue_(false) {}

void UartEncoderDriveBackend::begin(const RuntimeConfig &runtime) {
    (void)runtime;
    serial_.begin(config::MotorBoardBaud);
    startedAtMs_ = millis();
    lastZeroAtMs_ = startedAtMs_ - 50UL;
    pendingZero_ = true;
    serviceTransmit(startedAtMs_, runtime);
}

bool UartEncoderDriveBackend::tryWrite(const char *data, uint8_t length) {
    if (serial_.availableForWrite() < length) return false;
    return serial_.write(reinterpret_cast<const uint8_t *>(data), length) == length;
}

bool UartEncoderDriveBackend::tryWriteLiteral(const __FlashStringHelper *value, uint8_t length) {
    if (serial_.availableForWrite() < length) return false;
    serial_.print(value);
    return true;
}

uint8_t UartEncoderDriveBackend::appendUnsigned(char *output, uint8_t offset, uint32_t value) {
    char reverse[10];
    uint8_t length = 0;
    do { reverse[length++] = static_cast<char>('0' + value % 10UL); value /= 10UL; } while (value && length < sizeof(reverse));
    while (length > 0) output[offset++] = reverse[--length];
    return offset;
}

uint8_t UartEncoderDriveBackend::appendFixedMps(char *output, uint8_t offset, int16_t value) {
    int32_t signedValue = value;
    if (signedValue < 0) { output[offset++] = '-'; signedValue = -signedValue; }
    offset = appendUnsigned(output, offset, static_cast<uint32_t>(signedValue / 1000L));
    output[offset++] = '.';
    const uint16_t fraction = static_cast<uint16_t>(signedValue % 1000L);
    output[offset++] = static_cast<char>('0' + fraction / 100U);
    output[offset++] = static_cast<char>('0' + (fraction / 10U) % 10U);
    output[offset++] = static_cast<char>('0' + fraction % 10U);
    return offset;
}

uint8_t UartEncoderDriveBackend::appendPercent(char *output, uint8_t offset, int16_t value) {
    int32_t signedValue = value;
    if (signedValue < 0) { output[offset++] = '-'; signedValue = -signedValue; }
    offset = appendUnsigned(output, offset, static_cast<uint32_t>(signedValue / 100L));
    output[offset++] = '.';
    output[offset++] = static_cast<char>('0' + (signedValue / 10L) % 10L);
    output[offset++] = static_cast<char>('0' + signedValue % 10L);
    return offset;
}

bool UartEncoderDriveBackend::sendTargets(const RuntimeConfig &runtime) {
    char frame[64];
    uint8_t length = 0;
#if defined(ROBOT_UART_OPEN_LOOP)
    const char prefix[] = "$Car_Pwm:";
#else
    const char prefix[] = "$Car:";
#endif
    memcpy(frame, prefix, sizeof(prefix) - 1);
    length = sizeof(prefix) - 1;
    const int16_t logical[4] = {
        targets_.frontLeft, targets_.frontRight, targets_.rearLeft, targets_.rearRight
    };
    int16_t values[4] = {0, 0, 0, 0};
    for (uint8_t logicalIndex = 0; logicalIndex < 4; ++logicalIndex) {
        const uint8_t boardIndex = static_cast<uint8_t>(
            runtime.encoder.commandMap[logicalIndex]
        );
        values[boardIndex] = static_cast<int16_t>(
            logical[logicalIndex] * runtime.encoder.commandSigns[logicalIndex]
        );
    }
    for (uint8_t index = 0; index < 4; ++index) {
        feedback_.controllerTargetMmS[index] = values[index];
#if defined(ROBOT_UART_OPEN_LOOP)
        const MotorCalibration &calibration = runtime.uartOpenLoop[index];
        const int32_t magnitude = absolute32(values[index]);
        int32_t centiPercent = 0;
        if (magnitude > 0) {
            centiPercent = static_cast<int32_t>(calibration.minimumPwm) * 100L +
                magnitude * (calibration.maximumPwm - calibration.minimumPwm) *
                    100L / runtime.chassis.maximumWheelMmS;
            if (values[index] < 0) centiPercent = -centiPercent;
            centiPercent *= calibration.direction;
        }
        feedback_.openLoopPwm[index] = centiPercent;
        length = appendPercent(frame, length, static_cast<int16_t>(centiPercent));
#else
        feedback_.openLoopPwm[index] = 0;
        length = appendFixedMps(frame, length, values[index]);
#endif
        frame[length++] = index == 3 ? '!' : ',';
    }
    return tryWrite(frame, length);
}

void UartEncoderDriveBackend::serviceInitialization(uint32_t nowMs) {
    const uint32_t age = nowMs - startedAtMs_;
    if (initStage_ == InitStage::Settling && age >= 100UL) initStage_ = InitStage::MotorType;
    if (initStage_ == InitStage::MotorType) {
        if (tryWriteLiteral(F("$MOTOR_4CH_SET:0!"), 17)) initStage_ = InitStage::EncoderPolarity;
        return;
    }
    if (initStage_ == InitStage::EncoderPolarity && age >= 200UL) {
        if (tryWriteLiteral(F("$MOTOR_4CH_SET_ENCPDER_POLARITY:0!"), 34))
            initStage_ = InitStage::QualificationDelay;
    }
    if (initStage_ == InitStage::QualificationDelay && age >= 300UL)
        initStage_ = InitStage::Qualifying;
    if (initStage_ == InitStage::Qualifying && age >= 1300UL && consecutiveValid_ < 3) {
        faults_ |= FaultDriveInitialization;
    }
}

void UartEncoderDriveBackend::serviceTransmit(uint32_t nowMs, const RuntimeConfig &runtime) {
    if (!armed_ && elapsed(nowMs, lastZeroAtMs_, 50UL)) pendingZero_ = true;
    if (pendingZero_) {
        const char zero[] = "$Car:0,0,0,0!";
        if (tryWrite(zero, sizeof(zero) - 1)) {
            pendingZero_ = false;
            pendingMotor_ = false;
            lastZeroAtMs_ = nowMs;
            lastMotorCommandAtMs_ = nowMs;
        }
        return;
    }
    serviceInitialization(nowMs);
    if (pendingMotor_ && armed_ && initStage_ == InitStage::Ready) {
        if (sendTargets(runtime)) {
            pendingMotor_ = false;
            lastMotorCommandAtMs_ = nowMs;
        }
    }
}

void UartEncoderDriveBackend::serviceQuery(uint32_t nowMs) {
    if (outstanding_ != QueryType::None) return;
    if (encoderDue_) {
        if (tryWriteLiteral(F("$MOTOR_4CH_READ:encoder_20ms!"), 29)) {
            outstanding_ = QueryType::EncoderIncrement;
            querySentAtMs_ = nowMs;
            encoderDue_ = false;
        }
        return;
    }
    if (nowMs - lastEncoderCompletedAtMs_ > 5UL ||
        nowMs - lastEncoderDeadlineAtMs_ > 5UL) return;
    if (totalDue_) {
        if (tryWriteLiteral(F("$MOTOR_4CH_READ:encoder_total!"), 30)) {
            outstanding_ = QueryType::EncoderTotal;
            querySentAtMs_ = nowMs;
            totalDue_ = false;
        }
    } else if (batteryDue_) {
        if (tryWriteLiteral(F("$MOTOR_4CH_READ:battery!"), 24)) {
            outstanding_ = QueryType::Battery;
            querySentAtMs_ = nowMs;
            batteryDue_ = false;
        }
    }
}

void UartEncoderDriveBackend::pollReceive(uint32_t nowMs, const RuntimeConfig &runtime) {
    uint8_t processed = 0;
    while (serial_.available() > 0 && processed++ < 48) {
        const char value = static_cast<char>(serial_.read());
        if (parser_.feed(value) == MotorBoardFeedResult::Complete)
            finishMessage(parser_.frame(), nowMs, runtime);
    }
    if (outstanding_ != QueryType::None && nowMs - querySentAtMs_ >= 15UL) {
        if (outstanding_ == QueryType::EncoderIncrement) markMalformed();
        outstanding_ = QueryType::None;
        saturatingIncrement(queryTimeouts_);
    }
    if (initStage_ == InitStage::Ready && feedback_.incrementUpdatedAtMs != 0 &&
        nowMs - feedback_.incrementUpdatedAtMs > config::FeedbackStaleMs)
        faults_ |= FaultEncoderStale;
}

void UartEncoderDriveBackend::service(
    uint32_t nowMs, const RuntimeConfig &runtime
) {
    serviceTransmit(nowMs, runtime);
    if (!pendingZero_ && !pendingMotor_) serviceQuery(nowMs);
}

void UartEncoderDriveBackend::markMalformed() {
    if (consecutiveMalformed_ != 255) ++consecutiveMalformed_;
    consecutiveValid_ = 0;
    if (consecutiveMalformed_ >= 3) faults_ |= FaultEncoderMalformed;
}

void UartEncoderDriveBackend::finishMessage(
    const char *message, uint32_t nowMs, const RuntimeConfig &runtime
) {
    const char incrementPrefix[] = "$MOTOR_4CH_Encoder_20ms:";
    const char totalPrefix[] = "$MOTOR_4CH_Encoder_Total:";
    const char batteryPrefix[] = "$MOTOR_4CH_Battery:";
    int32_t values[4];
    if (outstanding_ == QueryType::EncoderIncrement &&
        strncmp(message, incrementPrefix, sizeof(incrementPrefix) - 1) == 0) {
        if (MotorBoardFrameParser::parseFour(
                message + sizeof(incrementPrefix) - 1, values, -32768L, 32767L
            ))
            acceptEncoder(values, nowMs, runtime);
        else markMalformed();
        outstanding_ = QueryType::None;
    } else if (outstanding_ == QueryType::EncoderTotal &&
               strncmp(message, totalPrefix, sizeof(totalPrefix) - 1) == 0) {
        if (MotorBoardFrameParser::parseFour(
                message + sizeof(totalPrefix) - 1, values, INT32_MIN, INT32_MAX
            ))
            acceptTotals(values, nowMs, runtime);
        outstanding_ = QueryType::None;
    } else if (outstanding_ == QueryType::Battery &&
               strncmp(message, batteryPrefix, sizeof(batteryPrefix) - 1) == 0) {
        const char *cursor = message + sizeof(batteryPrefix) - 1;
        int32_t value = 0;
        if (MotorBoardFrameParser::parseOne(cursor, value, 0, 65535L)) {
            feedback_.batteryMv = static_cast<uint16_t>(value);
            feedback_.batteryUpdatedAtMs = nowMs;
            feedback_.batteryValid = true;
        }
        outstanding_ = QueryType::None;
    }
}

void UartEncoderDriveBackend::acceptEncoder(
    const int32_t *values, uint32_t nowMs, const RuntimeConfig &runtime
) {
    uint16_t intervalMs = 20;
    if (runtime.encoder.semantics == EncoderSampleSemantics::ElapsedBetweenSamples &&
        feedback_.incrementUpdatedAtMs != 0) {
        const uint32_t actual = nowMs - feedback_.incrementUpdatedAtMs;
        if (actual < 10UL || actual > 100UL) { markMalformed(); return; }
        intervalMs = static_cast<uint16_t>(actual);
    }
    int16_t candidate[4];
    bool implausible = false;
    for (uint8_t logical = 0; logical < 4; ++logical) {
        const uint8_t channel = static_cast<uint8_t>(runtime.encoder.channelMap[logical]);
        if (values[channel] < -32768L || values[channel] > 32767L) { markMalformed(); return; }
        feedback_.rawIncrement[channel] = static_cast<int16_t>(values[channel]);
        const int64_t numerator = static_cast<int64_t>(values[channel]) *
            runtime.encoder.signs[logical] * runtime.encoder.wheelDiameterMm * 31416LL * 1000LL;
        const int64_t denominator = static_cast<int64_t>(runtime.encoder.countsPerRevolution) *
            10000LL * intervalMs;
        const int32_t speed = denominator == 0 ? 0 : static_cast<int32_t>(numerator / denominator);
        candidate[logical] = static_cast<int16_t>(speed > 32767 ? 32767 : speed < -32768 ? -32768 : speed);
        if (absolute32(candidate[logical]) > 1500L ||
            absolute32(static_cast<int32_t>(candidate[logical]) - previousMeasuredMmS_[logical]) > 1000L)
            implausible = true;
    }
    if (implausible) {
        if (implausibleSamples_ != 255) ++implausibleSamples_;
        if (implausibleSamples_ >= 2) faults_ |= FaultEncoderImplausible;
        return;
    }
    implausibleSamples_ = 0;
    for (uint8_t index = 0; index < 4; ++index) {
        feedback_.measuredMmS[index] = candidate[index];
        previousMeasuredMmS_[index] = candidate[index];
    }
    feedback_.sampleIntervalMs = intervalMs;
    feedback_.semantics = runtime.encoder.semantics;
    feedback_.incrementUpdatedAtMs = nowMs;
    feedback_.encoderValidMask = 0x0F;
    feedback_.errorValidMask = 0x0F;
    consecutiveMalformed_ = 0;
    if (consecutiveValid_ != 255) ++consecutiveValid_;
    lastEncoderCompletedAtMs_ = nowMs;
    if (consecutiveValid_ >= 3 && initStage_ == InitStage::Qualifying) initStage_ = InitStage::Ready;
    for (uint8_t index = 0; index < 4; ++index) updateWheelHealth(index, nowMs);
}

void UartEncoderDriveBackend::updateWheelHealth(uint8_t wheel, uint32_t nowMs) {
    const int16_t targetValues[4] = {targets_.frontLeft, targets_.frontRight, targets_.rearLeft, targets_.rearRight};
    const int16_t target = targetValues[wheel];
    const int16_t measured = feedback_.measuredMmS[wheel];
    feedback_.errorMmS[wheel] = static_cast<int16_t>(target - measured);
    if (absolute32(target) < 100L) {
        badSignSinceMs_[wheel] = stallSinceMs_[wheel] = mismatchSinceMs_[wheel] = 0;
        motionStartedAtMs_[wheel] = 0;
        previousTargetMmS_[wheel] = target;
        return;
    }
    if (absolute32(previousTargetMmS_[wheel]) < 100L ||
        ((previousTargetMmS_[wheel] < 0) != (target < 0)))
        motionStartedAtMs_[wheel] = nowMs;
    previousTargetMmS_[wheel] = target;
    const bool signBad = absolute32(measured) >= 50L && ((target < 0) != (measured < 0));
    if (signBad) {
        if (badSignSinceMs_[wheel] == 0) badSignSinceMs_[wheel] = nowMs;
        else if (nowMs - badSignSinceMs_[wheel] >= 250UL) {
#if ROBOT_DRIVE_QUALIFICATION
            warnings_ |= WarningEncoderSignCandidate;
#else
            faults_ |= FaultEncoderSign;
#endif
        }
    } else badSignSinceMs_[wheel] = 0;
    const bool settled = motionStartedAtMs_[wheel] != 0 &&
        nowMs - motionStartedAtMs_[wheel] >= 250UL;
    if (settled && absolute32(measured) <= 20L) {
        if (stallSinceMs_[wheel] == 0) stallSinceMs_[wheel] = nowMs;
        else if (nowMs - stallSinceMs_[wheel] >= 500UL) {
#if ROBOT_DRIVE_QUALIFICATION
            warnings_ |= WarningEncoderScaleCandidate;
#else
            faults_ |= FaultDriveStall;
#endif
        }
    } else stallSinceMs_[wheel] = 0;
    const int32_t allowedError = absolute32(target) / 2L > 100L ? absolute32(target) / 2L : 100L;
    if (absolute32(static_cast<int32_t>(target) - measured) > allowedError) {
        if (mismatchSinceMs_[wheel] == 0) mismatchSinceMs_[wheel] = nowMs;
        else if (nowMs - mismatchSinceMs_[wheel] >= 750UL) {
#if ROBOT_DRIVE_QUALIFICATION
            warnings_ |= WarningEncoderScaleCandidate;
#else
            faults_ |= FaultDriveMismatch;
#endif
        }
    } else mismatchSinceMs_[wheel] = 0;
}

void UartEncoderDriveBackend::acceptTotals(
    const int32_t *values, uint32_t nowMs, const RuntimeConfig &runtime
) {
    const int16_t logicalTargets[4] = {
        targets_.frontLeft, targets_.frontRight, targets_.rearLeft, targets_.rearRight
    };
    const bool hadTotals = feedback_.totalValidMask == 0x0F;
    for (uint8_t logical = 0; logical < 4; ++logical) {
        const uint8_t channel = static_cast<uint8_t>(runtime.encoder.channelMap[logical]);
        const bool settled = motionStartedAtMs_[logical] != 0 &&
            nowMs - motionStartedAtMs_[logical] >= 250UL;
        if (hadTotals && settled && absolute32(logicalTargets[logical]) >= 100L &&
            values[channel] == feedback_.total[channel]) {
            if (unchangedTotalFrames_[logical] != 255)
                ++unchangedTotalFrames_[logical];
            if (unchangedTotalFrames_[logical] >= 1) {
#if ROBOT_DRIVE_QUALIFICATION
                warnings_ |= WarningEncoderScaleCandidate;
#else
                faults_ |= FaultDriveStall;
#endif
            }
        } else {
            unchangedTotalFrames_[logical] = 0;
        }
    }
    for (uint8_t index = 0; index < 4; ++index)
        feedback_.total[index] = values[index];
    feedback_.totalUpdatedAtMs = nowMs;
    feedback_.totalValidMask = 0x0F;
}

void UartEncoderDriveBackend::setWheelTargets(const WheelTargets &targets) { targets_ = targets; }
void UartEncoderDriveBackend::onMotorDeadline(uint32_t nowMs, bool armed, const RuntimeConfig &runtime) {
    armed_ = armed;
    if (armed && initStage_ == InitStage::Ready) pendingMotor_ = true;
    else pendingZero_ = true;
    serviceTransmit(nowMs, runtime);
}
void UartEncoderDriveBackend::onEncoderDeadline(uint32_t nowMs, const RuntimeConfig &) {
    lastEncoderDeadlineAtMs_ = nowMs;
    if (initStage_ == InitStage::Qualifying || initStage_ == InitStage::Ready) encoderDue_ = true;
}
void UartEncoderDriveBackend::onEncoderTotalDeadline(uint32_t nowMs) { totalDue_ = true; serviceQuery(nowMs); }
void UartEncoderDriveBackend::onBatteryDeadline(uint32_t nowMs) { batteryDue_ = true; serviceQuery(nowMs); }
void UartEncoderDriveBackend::stop(uint32_t nowMs) {
    armed_ = false;
    targets_ = {0, 0, 0, 0};
    for (uint8_t index = 0; index < 4; ++index) {
        feedback_.controllerTargetMmS[index] = 0;
        feedback_.openLoopPwm[index] = 0;
    }
    pendingMotor_ = false;
    pendingZero_ = true;
    const char zero[] = "$Car:0,0,0,0!";
    if (tryWrite(zero, sizeof(zero) - 1)) {
        pendingZero_ = false;
        lastZeroAtMs_ = nowMs;
        lastMotorCommandAtMs_ = nowMs;
    }
}

DriveCapabilities UartEncoderDriveBackend::capabilities() const {
#if defined(ROBOT_UART_OPEN_LOOP)
    return {DriveControlMode::UartOpenLoopPwm, PwmUnit::PercentX100, true, true, false};
#else
    return {DriveControlMode::UartClosedLoopSpeed, PwmUnit::Unavailable, true, true, false};
#endif
}
const DriveFeedback &UartEncoderDriveBackend::feedback() const { return feedback_; }
DriveHealth UartEncoderDriveBackend::health(uint32_t nowMs) const {
    const bool fresh = feedback_.incrementUpdatedAtMs != 0 &&
        nowMs - feedback_.incrementUpdatedAtMs <= config::FeedbackStaleMs;
    const bool ready = initStage_ == InitStage::Ready && consecutiveValid_ >= 3;
    uint16_t warnings = warnings_;
    if (!config::DriveCalibrationQualified) warnings |= WarningDriveUnqualified;
    return {faults_, warnings, initStage_ == InitStage::Ready, ready, ready && fresh};
}
void UartEncoderDriveBackend::clearFaults() {
    faults_ = 0;
    consecutiveMalformed_ = 0;
    warnings_ = config::DriveCalibrationQualified ? 0 : WarningDriveUnqualified;
}
uint16_t UartEncoderDriveBackend::queryTimeouts() const { return queryTimeouts_; }
uint16_t UartEncoderDriveBackend::rxOverflows() const { return parser_.overflows(); }
uint16_t UartEncoderDriveBackend::motorCommandAgeMs(uint32_t nowMs) const {
    const uint32_t age = nowMs - lastMotorCommandAtMs_;
    return static_cast<uint16_t>(age > 65535UL ? 65535UL : age);
}
uint8_t UartEncoderDriveBackend::outstandingQuery() const {
    return static_cast<uint8_t>(outstanding_);
}
uint16_t UartEncoderDriveBackend::outstandingQueryAgeMs(uint32_t nowMs) const {
    if (outstanding_ == QueryType::None) return 0;
    const uint32_t age = nowMs - querySentAtMs_;
    return static_cast<uint16_t>(age > 65535UL ? 65535UL : age);
}

} // namespace robot
