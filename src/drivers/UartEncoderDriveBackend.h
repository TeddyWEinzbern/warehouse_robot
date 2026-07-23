#pragma once

#include <Arduino.h>

#include "core/MotorBoardProtocol.h"
#include "drivers/DriveBackend.h"

namespace robot {

class UartEncoderDriveBackend : public DriveBackend {
  public:
    explicit UartEncoderDriveBackend(HardwareSerial &serial);
    void begin(const RuntimeConfig &runtime);
    void pollReceive(uint32_t nowMs, const RuntimeConfig &runtime);
    void service(uint32_t nowMs, const RuntimeConfig &runtime);
    void setWheelTargets(const WheelTargets &targets);
    void onMotorDeadline(uint32_t nowMs, bool armed, const RuntimeConfig &runtime);
    void onEncoderDeadline(uint32_t nowMs, const RuntimeConfig &runtime);
    void onEncoderTotalDeadline(uint32_t nowMs);
    void onBatteryDeadline(uint32_t nowMs);
    void stop(uint32_t nowMs);
#if ROBOT_CALIBRATION
    // Spins one motor board channel while DISARMED for the calibration
    // profile: mode 0 sends $Car_Pwm percent, mode 1 sends $Car mm/s. The
    // frame repeats at the keepalive cadence and reverts to the zero frame
    // once durationMs elapses. Returns false until the board is initialized.
    bool startCalibrationSpin(
        uint8_t mode, uint8_t channel, int16_t value,
        uint16_t durationMs, uint32_t nowMs
    );
#endif
    DriveCapabilities capabilities() const;
    const DriveFeedback &feedback() const;
    DriveHealth health(uint32_t nowMs) const;
    void clearFaults();
    uint16_t queryTimeouts() const;
    uint16_t rxOverflows() const;
    uint16_t motorCommandAgeMs(uint32_t nowMs) const;
    uint8_t outstandingQuery() const;
    uint16_t outstandingQueryAgeMs(uint32_t nowMs) const;

  private:
    enum class InitStage : uint8_t {
        Settling, MotorType, EncoderPolarity, QualificationDelay, Qualifying, Ready
    };
    enum class QueryType : uint8_t { None, EncoderIncrement, EncoderTotal, Battery };

    HardwareSerial &serial_;
    DriveFeedback feedback_;
    WheelTargets targets_;
    InitStage initStage_;
    QueryType outstanding_;
    uint32_t startedAtMs_;
    uint32_t querySentAtMs_;
    uint32_t lastZeroAtMs_;
    uint32_t lastMotorCommandAtMs_;
    uint32_t lastEncoderCompletedAtMs_;
    uint32_t lastEncoderDeadlineAtMs_;
    uint32_t badSignSinceMs_[4];
    uint32_t stallSinceMs_[4];
    uint32_t mismatchSinceMs_[4];
    uint32_t motionStartedAtMs_[4];
    int16_t previousTargetMmS_[4];
    int16_t previousMeasuredMmS_[4];
    uint16_t faults_;
    uint16_t warnings_;
    uint16_t queryTimeouts_;
    uint8_t consecutiveValid_;
    uint8_t consecutiveMalformed_;
    uint8_t implausibleSamples_;
    uint8_t unchangedTotalFrames_[4];
    MotorBoardFrameParser parser_;
#if ROBOT_CALIBRATION
    uint32_t calibrationUntilMs_;
    int16_t calibrationValue_;
    uint8_t calibrationChannel_;
    bool calibrationOpenLoop_;
    bool calibrationActive_;
    bool sendCalibrationFrame();
#endif
    bool armed_;
    bool pendingZero_;
    bool pendingMotor_;
    bool encoderDue_;
    bool totalDue_;
    bool batteryDue_;

    bool tryWrite(const char *data, uint8_t length);
    bool tryWriteLiteral(const __FlashStringHelper *value, uint8_t length);
    void serviceInitialization(uint32_t nowMs);
    void serviceTransmit(uint32_t nowMs, const RuntimeConfig &runtime);
    void serviceQuery(uint32_t nowMs);
    bool sendTargets(const RuntimeConfig &runtime);
    static uint8_t appendUnsigned(char *output, uint8_t offset, uint32_t value);
    static uint8_t appendFixedMps(char *output, uint8_t offset, int16_t millimetresPerSecond);
    static uint8_t appendPercent(char *output, uint8_t offset, int16_t centiPercent);
    void finishMessage(
        const char *message,
        uint32_t nowMs,
        const RuntimeConfig &runtime
    );
    void acceptEncoder(const int32_t *values, uint32_t nowMs, const RuntimeConfig &runtime);
    void acceptTotals(
        const int32_t *values,
        uint32_t nowMs,
        const RuntimeConfig &runtime
    );
    void updateWheelHealth(uint8_t wheel, uint32_t nowMs);
    void markMalformed();
};

} // namespace robot
