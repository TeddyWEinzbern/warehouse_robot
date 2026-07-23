#include "app/RobotApplication.h"

#include "app/PinProfile.h"

namespace robot {
namespace {
void putU16(uint8_t *output, uint8_t &offset, uint16_t value) {
    output[offset++] = static_cast<uint8_t>(value);
    output[offset++] = static_cast<uint8_t>(value >> 8);
}

void putI16(uint8_t *output, uint8_t &offset, int16_t value) {
    putU16(output, offset, static_cast<uint16_t>(value));
}

void putU32(uint8_t *output, uint8_t &offset, uint32_t value) {
    output[offset++] = static_cast<uint8_t>(value);
    output[offset++] = static_cast<uint8_t>(value >> 8);
    output[offset++] = static_cast<uint8_t>(value >> 16);
    output[offset++] = static_cast<uint8_t>(value >> 24);
}

void saturatingIncrement(uint16_t &value) {
    if (value != 65535U) ++value;
}

uint16_t saturatingAge(uint32_t nowMs, uint32_t updatedAtMs) {
    if (updatedAtMs == 0) return 65535U;
    const uint32_t age = nowMs - updatedAtMs;
    return static_cast<uint16_t>(age > 65535UL ? 65535UL : age);
}
} // namespace

RobotApplication::RobotApplication()
    : bluetooth_(pins::BluetoothRx, pins::BluetoothTx), communication_(),
#if defined(ROBOT_BACKEND_UART)
      driveBackend_(Serial),
#else
      driveBackend_(),
#endif
      chassis_(driveBackend_), runtime_(RuntimeConfig::defaults()), activeFrame_({}),
      assistOutput_({{0, 0, 0, 0, IntentSource::Safety}, 0.0F,
                     AssistStage::Idle, false, false}),
      status_({}), missWindowStartedUs_(0),
      motorMissBaseline_(0), chassisMissBaseline_(0), droppedTelemetry_(0),
      motionDtClamps_(0), consecutiveMotorLate_(0),
      lastProcessedControlSequence_(255), telemetrySequence_(0),
      telemetryDetailSlot_(0), snapshotCursor_(0), snapshotSequence_(0),
      snapshotActive_(false), helloPending_(false), platformInitialized_(false),
      previousState_(RobotState::Boot) {}

Stream &RobotApplication::hostStream() {
    // The hardware UART (D0/D1) always belongs to the motor board; the host
    // link is the A4/A5 SoftwareSerial in every profile, calibration
    // included (HC-05 or USB-TTL adapter).
    return bluetooth_;
}

RobotProfile RobotApplication::profile() const {
#if ROBOT_CALIBRATION
    return RobotProfile::Calibration;
#elif defined(ROBOT_UART_OPEN_LOOP)
    return RobotProfile::UartOpenLoopRobot;
#else
    return RobotProfile::UartClosedLoopRobot;
#endif
}

bool RobotApplication::profileCanArm() const {
#if ROBOT_CALIBRATION
    return false;
#else
    return config::DriveCalibrated;
#endif
}

bool RobotApplication::sonarEnabled() const { return true; }

bool RobotApplication::armMotionEnabled() const {
#if ROBOT_CALIBRATION
    return false;
#else
    return arm_.calibrated();
#endif
}

void RobotApplication::begin() {
    bluetooth_.begin(config::BluetoothBaud);
    chassis_.begin(runtime_);
    arm_.begin(runtime_);
    if (sonarEnabled()) sensors_.begin();

    const uint32_t nowUs = micros();
    chassisTask_.start(nowUs, config::ChassisRampPeriodUs, 0);
    motorTask_.start(nowUs, config::MotorCommandPeriodUs, 0);
    encoderTask_.start(nowUs, config::EncoderQueryPeriodUs, 5000UL);
    servoTask_.start(nowUs, config::ServoPeriodUs, 10000UL);
    sonarTask_.start(nowUs, config::SonarGroupPeriodUs, 15000UL);
    encoderTotalTask_.start(nowUs, config::EncoderTotalPeriodUs, 250000UL);
    batteryTask_.start(nowUs, config::BatteryPeriodUs, 750000UL);
    telemetryTask_.start(nowUs, config::TelemetryPeriodUs, config::TelemetryPeriodUs);
    schedulerHealth_.begin(nowUs);
    missWindowStartedUs_ = nowUs;
    platformInitialized_ = true;
}

void RobotApplication::processHostMessages(
    uint32_t, const ControlRequests &requests
) {
    if ((requests.flags & RequestHello) != 0) helloPending_ = true;
    uint8_t requestedSequence = 0;
    if (communication_.takeSnapshotRequest(requestedSequence)) {
        snapshotActive_ = true;
        snapshotCursor_ = 0;
        snapshotSequence_ = requestedSequence;
    }

    PendingParameter parameter = {};
    if (communication_.transmitIdle() && communication_.takeParameter(parameter)) {
        if (safety_.state() != RobotState::Disarmed) {
            communication_.sendNack(
                hostStream(), parameter.sequence, MessageType::ParameterSet,
                NackReason::InvalidState
            );
        } else if (parameter.expectedRevision != runtime_.revision) {
            communication_.sendNack(
                hostStream(), parameter.sequence, MessageType::ParameterSet,
                NackReason::RevisionMismatch
            );
        } else if (!runtime_.applyParameter(
                       parameter.group, parameter.index, parameter.data,
                       parameter.length, config::DriveCalibrated
                   )) {
            communication_.sendNack(
                hostStream(), parameter.sequence, MessageType::ParameterSet,
                NackReason::ValidationFailed
            );
        } else {
            communication_.sendAck(
                hostStream(), parameter.sequence, MessageType::ParameterSet,
                runtime_.revision
            );
        }
    }

    PendingCalibration calibration = {};
    if (communication_.transmitIdle() && communication_.takeCalibration(calibration)) {
#if ROBOT_CALIBRATION
        if (safety_.state() != RobotState::Disarmed || calibration.joint >= 4 ||
            calibration.degrees > 180) {
            communication_.sendNack(
                hostStream(), calibration.sequence,
                MessageType::CalibrationCommand, NackReason::InvalidState
            );
        } else if (!arm_.setCalibrationJoint(
                       calibration.joint, calibration.degrees, runtime_
                   )) {
            // Rejected by the joint-coupling guard (four-bar fold band).
            communication_.sendNack(
                hostStream(), calibration.sequence,
                MessageType::CalibrationCommand, NackReason::ValidationFailed
            );
        } else {
            communication_.sendAck(
                hostStream(), calibration.sequence,
                MessageType::CalibrationCommand, runtime_.revision
            );
        }
#else
        communication_.sendNack(
            hostStream(), calibration.sequence,
            MessageType::CalibrationCommand, NackReason::InvalidState
        );
#endif
    }

    PendingDriveCalibration spin = {};
    if (communication_.transmitIdle() && communication_.takeDriveCalibration(spin)) {
#if ROBOT_CALIBRATION
        const int16_t magnitude = spin.value < 0
            ? static_cast<int16_t>(-spin.value) : spin.value;
        const int16_t limit = spin.mode == 0
            ? config::CalibrationSpinLimitPercent
            : config::CalibrationWheelLimitMmS;
        if (safety_.state() != RobotState::Disarmed) {
            communication_.sendNack(
                hostStream(), spin.sequence,
                MessageType::DriveCalibrationCommand, NackReason::InvalidState
            );
        } else if (spin.mode > 1 || spin.channel > 3 || magnitude > limit ||
                   spin.durationMs > config::CalibrationSpinMaxDurationMs) {
            communication_.sendNack(
                hostStream(), spin.sequence,
                MessageType::DriveCalibrationCommand, NackReason::ValidationFailed
            );
        } else if (!driveBackend_.startCalibrationSpin(
                       spin.mode, spin.channel, spin.value, spin.durationMs,
                       millis()
                   )) {
            // Motor board not initialized yet (still settling or faulted).
            communication_.sendNack(
                hostStream(), spin.sequence,
                MessageType::DriveCalibrationCommand, NackReason::InvalidState
            );
        } else {
            communication_.sendAck(
                hostStream(), spin.sequence,
                MessageType::DriveCalibrationCommand, runtime_.revision
            );
        }
#else
        communication_.sendNack(
            hostStream(), spin.sequence,
            MessageType::DriveCalibrationCommand, NackReason::InvalidState
        );
#endif
    }
}

void RobotApplication::enforceSafetyStop(uint32_t nowMs) {
    if (!safety_.takeImmediateStop()) return;
    chassis_.forceZero(nowMs);
#if !ROBOT_CALIBRATION
    assist_.cancel();
#endif
    arm_.holdLastCommanded();
}

void RobotApplication::updateMotionIntent(uint32_t nowMs) {
#if ROBOT_CALIBRATION
    // The calibration profile can never arm: motors only move through the
    // DISARMED calibration spin path and the backend's zero keepalive, so
    // the whole motion-intent pipeline (assist, presets, chassis ramp) is
    // compiled out to keep the image inside flash.
    (void)nowMs;
#else
    const bool motionAllowed = safety_.armed();
    if (!motionAllowed) {
        assist_.cancel();
        chassis_.setDesired(
            {0, 0, 0, 0, IntentSource::Safety}, runtime_
        );
        return;
    }

    arm_.requestPreset(activeFrame_.pressed, runtime_);
    assistOutput_ = assist_.update(
        activeFrame_, sensors_.snapshot(), arm_.currentTarget(),
        armMotionEnabled(), nowMs, runtime_
    );
    if (assistOutput_.reachActive && armMotionEnabled())
        arm_.requestReach(assistOutput_.requestedReachMm, runtime_);
    const DriveIntent intent = safety_.arbitrate(
        activeFrame_, assistOutput_, arm_.cargoMayBeHeld(), runtime_
    );
    chassis_.setDesired(intent, runtime_);
#endif
}

void RobotApplication::evaluateMissWindow(uint32_t nowUs) {
    if (nowUs - missWindowStartedUs_ < 1000000UL) return;
    const uint16_t motorMisses = static_cast<uint16_t>(
        motorTask_.stats().missed - motorMissBaseline_
    );
    const uint16_t chassisMisses = static_cast<uint16_t>(
        chassisTask_.stats().missed - chassisMissBaseline_
    );
    if (safety_.armed() && (motorMisses >= 3 || chassisMisses >= 5))
        safety_.latchFault(FaultSchedulerOverrun);
    motorMissBaseline_ = motorTask_.stats().missed;
    chassisMissBaseline_ = chassisTask_.stats().missed;
    missWindowStartedUs_ = nowUs;
}

void RobotApplication::runDueTasks(uint32_t nowMs, uint32_t nowUs) {
    if (chassisTask_.due(nowUs)) {
        uint32_t elapsedUs = chassisTask_.elapsedUs(nowUs, 0xFFFFFFFFUL);
        if (elapsedUs > config::MaxMotionDtUs) {
            elapsedUs = config::MaxMotionDtUs;
            saturatingIncrement(motionDtClamps_);
        }
#if !ROBOT_CALIBRATION
        if (safety_.armed()) chassis_.trajectoryTick(nowUs, elapsedUs, runtime_);
#endif
        if (safety_.armed() && chassisTask_.stats().consecutiveMisses >= 3)
            safety_.latchFault(FaultSchedulerOverrun);
        enforceSafetyStop(nowMs);
    }

    if (motorTask_.due(nowUs)) {
        if (motorTask_.lastLatenessUs() > config::MotorLateThresholdUs) {
            if (consecutiveMotorLate_ != 255) ++consecutiveMotorLate_;
        } else {
            consecutiveMotorLate_ = 0;
        }
        if (safety_.armed() && consecutiveMotorLate_ >= 2)
            safety_.latchFault(FaultSchedulerOverrun);
        enforceSafetyStop(nowMs);
#if !ROBOT_CALIBRATION
        if (safety_.armed()) chassis_.motorTick(nowMs, true, runtime_);
#endif
    }

    if (encoderTask_.due(nowUs))
        driveBackend_.onEncoderDeadline(nowMs, runtime_);

    if (servoTask_.due(nowUs)) {
        uint32_t elapsedUs = servoTask_.elapsedUs(nowUs, 0xFFFFFFFFUL);
        if (elapsedUs > config::MaxMotionDtUs) {
            elapsedUs = config::MaxMotionDtUs;
            saturatingIncrement(motionDtClamps_);
        }
#if ROBOT_CALIBRATION
        arm_.calibrationTick(elapsedUs, runtime_);
#else
        if (safety_.armed() && armMotionEnabled())
            arm_.update(activeFrame_, elapsedUs, runtime_);
#endif
        if (safety_.armed() && servoTask_.stats().consecutiveMisses >= 3)
            safety_.latchFault(FaultSchedulerOverrun);
        if (arm_.faulted()) safety_.latchFault(FaultArmTarget);
        enforceSafetyStop(nowMs);
    }

    if (sonarTask_.due(nowUs) && sonarEnabled()) {
        if (!sensors_.startNextGroup(nowMs, nowUs, runtime_))
            saturatingIncrement(droppedTelemetry_);
    }
    if (encoderTotalTask_.due(nowUs))
        driveBackend_.onEncoderTotalDeadline(nowMs);
    if (batteryTask_.due(nowUs))
        driveBackend_.onBatteryDeadline(nowMs);

    evaluateMissWindow(nowUs);
    enforceSafetyStop(nowMs);
    driveBackend_.service(nowMs, runtime_);

    if (telemetryTask_.due(nowUs)) {
        const bool motorImminent = safety_.armed() &&
            motorTask_.untilDeadlineUs(nowUs) < 12000UL;
        if (sensors_.capturing() || motorImminent) {
            saturatingIncrement(droppedTelemetry_);
        } else {
            sendTelemetry(nowMs);
        }
    }
}

void RobotApplication::update() {
    const uint32_t nowUs = micros();
    const uint32_t nowMs = millis();
    if (schedulerHealth_.observeLoop(nowUs, safety_.armed()))
        safety_.latchFault(FaultSchedulerOverrun);

    // Existing timeout/fault state is applied before either receive channel.
    const ControlRequests noRequests = {0};
    safety_.update(
        activeFrame_, noRequests, driveBackend_.health(nowMs),
        platformInitialized_, profileCanArm(), nowMs
    );
    enforceSafetyStop(nowMs);

    driveBackend_.pollReceive(nowMs, runtime_);
    communication_.poll(hostStream(), nowMs);
    if (sonarEnabled()) sensors_.poll(nowMs, nowUs);

    const OperatorControlFrame &received = communication_.latest();
    if (received.valid) {
        activeFrame_ = received;
        if (activeFrame_.sequence == lastProcessedControlSequence_)
            activeFrame_.pressed = 0;
        else
            lastProcessedControlSequence_ = activeFrame_.sequence;
    }

    const ControlRequests requests = communication_.takeRequests();
    if (arm_.faulted()) safety_.latchFault(FaultArmTarget);
    safety_.update(
        activeFrame_, requests, driveBackend_.health(nowMs),
        platformInitialized_, profileCanArm(), nowMs
    );
    if (safety_.takeClearFaultAccepted()) {
        driveBackend_.clearFaults();
        arm_.clearFault();
    }
    enforceSafetyStop(nowMs);
    processHostMessages(nowMs, requests);

    if (previousState_ != safety_.state()) {
        if (safety_.state() == RobotState::Armed) arm_.releaseHold();
        previousState_ = safety_.state();
    }
    const bool hostTransmitWindow =
#if ROBOT_CALIBRATION
        true;
#else
        activeFrame_.valid && nowMs - activeFrame_.receivedAtMs <=
            (config::BluetoothBaud == 9600UL ? 60UL : 30UL);
#endif
    if (hostTransmitWindow) {
        communication_.pumpTransmit(
            hostStream(), config::BluetoothBaud == 9600UL ? 2 : 8
        );
    }
    driveBackend_.service(nowMs, runtime_);

    updateMotionIntent(nowMs);
    activeFrame_.pressed = 0;
    runDueTasks(nowMs, nowUs);
}

void RobotApplication::sendHello() {
    const DriveCapabilities capabilities = driveBackend_.capabilities();
    uint8_t payload[8] = {
        static_cast<uint8_t>(profile()),
        static_cast<uint8_t>(capabilities.mode),
        static_cast<uint8_t>(capabilities.pwmUnit),
        static_cast<uint8_t>(
            (config::DriveCalibrated ? 1U : 0U) |
            (arm_.calibrated() ? 2U : 0U) |
            (ROBOT_CALIBRATION ? 4U : 0U) |
            (capabilities.encoderFeedback ? 8U : 0U)
        ),
        static_cast<uint8_t>(runtime_.revision),
        static_cast<uint8_t>(runtime_.revision >> 8),
        static_cast<uint8_t>(config::BluetoothBaud / 1200UL),
        0,
    };
    communication_.sendFrame(
        hostStream(), MessageType::HelloTelemetry, telemetrySequence_++,
        payload, sizeof(payload)
    );
}

void RobotApplication::sendStatus(uint32_t nowMs) {
    const DriveHealth drive = driveBackend_.health(nowMs);
    status_.state = safety_.state();
    status_.assistStage = assistOutput_.stage;
    status_.faults = static_cast<uint16_t>(safety_.faults() | drive.faults);
    status_.warnings = static_cast<uint16_t>(
        drive.warnings | (arm_.targetLimited() ? WarningArmTargetLimited : 0U)
    );
    status_.commandAgeMs = saturatingAge(nowMs, activeFrame_.receivedAtMs);
    status_.cargoMayBeHeld = arm_.cargoMayBeHeld();
    status_.linkAlive = safety_.linkAlive();
    status_.emergencyStopped = safety_.emergencyStopped();
    status_.driveCalibrated = config::DriveCalibrated;
    uint8_t payload[14];
    uint8_t offset = 0;
    payload[offset++] = static_cast<uint8_t>(status_.state);
    payload[offset++] = static_cast<uint8_t>(status_.assistStage);
    putU16(payload, offset, status_.faults);
    putU16(payload, offset, status_.warnings);
    putU16(payload, offset, status_.commandAgeMs);
    payload[offset++] = static_cast<uint8_t>(
        (status_.linkAlive ? 1U : 0U) |
        (status_.emergencyStopped ? 2U : 0U) |
        (status_.cargoMayBeHeld ? 4U : 0U) |
        (status_.driveCalibrated ? 8U : 0U)
    );
    payload[offset++] = static_cast<uint8_t>(runtime_.chassis.activeProfile);
    putU16(payload, offset, runtime_.revision);
    putU16(payload, offset, communication_.malformedFrames());
    communication_.sendFrame(
        hostStream(), MessageType::StatusTelemetry, telemetrySequence_++,
        payload, offset
    );
}

void RobotApplication::sendDriveCommand(uint32_t nowMs) {
    uint8_t payload[26];
    uint8_t offset = 0;
    payload[offset++] = static_cast<uint8_t>(driveBackend_.capabilities().mode);
    payload[offset++] = static_cast<uint8_t>(runtime_.chassis.activeProfile);
    payload[offset++] = chassis_.zeroCrossingMask(micros());
    const ChassisVelocity &requested = chassis_.requestedVelocity();
    const ChassisVelocity &ramped = chassis_.rampedVelocity();
    putI16(payload, offset, requested.longitudinalMmS);
    putI16(payload, offset, requested.lateralMmS);
    putI16(payload, offset, requested.yawMradS);
    putI16(payload, offset, ramped.longitudinalMmS);
    putI16(payload, offset, ramped.lateralMmS);
    putI16(payload, offset, ramped.yawMradS);
    const DriveFeedback &feedback = driveBackend_.feedback();
    for (uint8_t wheel = 0; wheel < 4; ++wheel)
        putI16(payload, offset, feedback.controllerTargetMmS[wheel]);
    payload[offset++] = driveBackend_.capabilities().pwmUnit != PwmUnit::Unavailable;
    putU16(payload, offset, driveBackend_.motorCommandAgeMs(nowMs));
    communication_.sendFrame(
        hostStream(), MessageType::DriveCommandTelemetry, telemetrySequence_++,
        payload, offset
    );
}

void RobotApplication::sendDriveFeedback(uint32_t nowMs) {
    const DriveFeedback &feedback = driveBackend_.feedback();
    const uint16_t feedbackAge = saturatingAge(nowMs, feedback.incrementUpdatedAtMs);
    const bool feedbackFresh = feedbackAge <= config::FeedbackStaleMs;
    uint8_t payload[27];
    uint8_t offset = 0;
    for (uint8_t wheel = 0; wheel < 4; ++wheel)
        putI16(payload, offset, feedback.measuredMmS[wheel]);
    for (uint8_t wheel = 0; wheel < 4; ++wheel)
        putI16(payload, offset, feedback.errorMmS[wheel]);
    payload[offset++] = feedbackFresh ? feedback.encoderValidMask : 0;
    payload[offset++] = feedbackFresh ? feedback.errorValidMask : 0;
    payload[offset++] = feedback.totalValidMask;
    putU16(payload, offset, feedbackAge);
    putU16(payload, offset, feedback.sampleIntervalMs);
    payload[offset++] = static_cast<uint8_t>(feedback.semantics);
    payload[offset++] = driveBackend_.outstandingQuery();
    putU16(payload, offset, driveBackend_.outstandingQueryAgeMs(nowMs));
    communication_.sendFrame(
        hostStream(), MessageType::DriveFeedbackTelemetry, telemetrySequence_++,
        payload, offset
    );
}

void RobotApplication::sendEncoderTotals(uint32_t nowMs) {
    const DriveFeedback &feedback = driveBackend_.feedback();
    uint8_t payload[27];
    uint8_t offset = 0;
    for (uint8_t wheel = 0; wheel < 4; ++wheel)
        putI16(payload, offset, feedback.rawIncrement[wheel]);
    for (uint8_t wheel = 0; wheel < 4; ++wheel)
        putU32(payload, offset, static_cast<uint32_t>(feedback.total[wheel]));
    payload[offset++] = feedback.totalValidMask;
    putU16(payload, offset, saturatingAge(nowMs, feedback.totalUpdatedAtMs));
    communication_.sendFrame(
        hostStream(), MessageType::EncoderTotalsTelemetry, telemetrySequence_++,
        payload, offset
    );
}

void RobotApplication::sendScheduler() {
    uint8_t payload[27];
    uint8_t offset = 0;
    putU32(payload, offset, schedulerHealth_.maxLoopGapUs());
    putU16(payload, offset, chassisTask_.stats().missed);
    putU16(payload, offset, motorTask_.stats().missed);
    putU16(payload, offset, encoderTask_.stats().missed);
    putU16(payload, offset, servoTask_.stats().missed);
    putU16(payload, offset, sonarTask_.stats().missed);
    putU16(payload, offset, telemetryTask_.stats().missed);
    putU16(payload, offset, driveBackend_.queryTimeouts());
    putU16(payload, offset, driveBackend_.rxOverflows());
    const uint32_t dropped = static_cast<uint32_t>(droppedTelemetry_) +
        communication_.txDrops();
    putU16(payload, offset, static_cast<uint16_t>(dropped > 65535UL ? 65535UL : dropped));
    putU16(payload, offset, motionDtClamps_);
    putU16(payload, offset, communication_.rxOverflows());
    communication_.sendFrame(
        hostStream(), MessageType::SchedulerTelemetry, telemetrySequence_++,
        payload, offset
    );
}

void RobotApplication::sendSensorArm(uint32_t nowMs) {
    uint8_t payload[24];
    uint8_t offset = 0;
    uint8_t validMask = 0;
    for (uint8_t direction = 0; direction < 3; ++direction) {
        const DistancePair &pair = sensors_.snapshot().directions[direction];
        const DistanceReading readings[2] = {pair.first, pair.second};
        for (uint8_t slot = 0; slot < 2; ++slot) {
            const uint8_t bit = static_cast<uint8_t>(direction * 2U + slot);
            const bool valid = readings[slot].valid &&
                nowMs - readings[slot].updatedAtMs <= config::SensorStaleMs;
            if (valid) validMask |= static_cast<uint8_t>(1U << bit);
            putU16(payload, offset, readings[slot].millimetres);
        }
    }
    payload[offset++] = validMask;
    const uint8_t *servos = arm_.lastCommandedDegrees();
    for (uint8_t joint = 0; joint < 4; ++joint) payload[offset++] = servos[joint];
    payload[offset++] = arm_.calibrated();
    payload[offset++] = arm_.cargoMayBeHeld();
    const DriveFeedback &feedback = driveBackend_.feedback();
    putU16(payload, offset, feedback.batteryMv);
    payload[offset++] = feedback.batteryValid;
    putU16(payload, offset, saturatingAge(nowMs, feedback.batteryUpdatedAtMs));
    communication_.sendFrame(
        hostStream(), MessageType::SensorArmTelemetry, telemetrySequence_++,
        payload, offset
    );
}

void RobotApplication::sendOpenLoopPwm() {
    const DriveCapabilities capabilities = driveBackend_.capabilities();
    if (capabilities.pwmUnit == PwmUnit::Unavailable) return;
    uint8_t payload[10];
    uint8_t offset = 0;
    payload[offset++] = static_cast<uint8_t>(capabilities.pwmUnit);
    payload[offset++] = 0x0F;
    const DriveFeedback &feedback = driveBackend_.feedback();
    for (uint8_t wheel = 0; wheel < 4; ++wheel)
        putI16(payload, offset, feedback.openLoopPwm[wheel]);
    communication_.sendFrame(
        hostStream(), MessageType::OpenLoopPwmTelemetry, telemetrySequence_++,
        payload, offset
    );
}

bool RobotApplication::sendNextParameterSnapshot() {
    if (!snapshotActive_) return false;
    uint8_t payload[27];
    uint8_t offset = 0;
    ParameterGroup group = ParameterGroup::Servo;
    uint8_t index = 0;
    uint8_t data[23];
    uint8_t length = 0;
    if (snapshotCursor_ < 4) {
        group = ParameterGroup::Servo; index = snapshotCursor_;
        const ServoCalibration &value = runtime_.servos[index];
        data[0] = value.lowerDegrees; data[1] = value.upperDegrees;
        data[2] = static_cast<uint8_t>(value.centerOffsetDegrees);
        data[3] = static_cast<uint8_t>(value.direction); length = 4;
    } else if (snapshotCursor_ < 8) {
        group = ParameterGroup::UartOpenLoop; index = snapshotCursor_ - 4;
        const MotorCalibration &value = runtime_.uartOpenLoop[index];
        data[0] = value.minimumPwm; data[1] = value.maximumPwm;
        data[2] = static_cast<uint8_t>(value.direction); length = 3;
    } else if (snapshotCursor_ == 8) {
        group = ParameterGroup::ChassisSpeed;
        int16_t values[5] = {
            runtime_.chassis.maximumForwardMmS,
            runtime_.chassis.maximumReverseMmS,
            runtime_.chassis.maximumLateralMmS,
            runtime_.chassis.maximumYawMradS,
            static_cast<int16_t>(runtime_.chassis.maximumWheelMmS)
        };
        for (uint8_t i = 0; i < 5; ++i) {
            data[length++] = static_cast<uint8_t>(values[i]);
            data[length++] = static_cast<uint8_t>(values[i] >> 8);
        }
    } else if (snapshotCursor_ == 9) {
        group = ParameterGroup::ChassisAcceleration; index = 0;
        const uint16_t values[8] = {
            runtime_.chassis.acceleration.forwardAccelMmS2,
            runtime_.chassis.acceleration.forwardDecelMmS2,
            runtime_.chassis.acceleration.reverseAccelMmS2,
            runtime_.chassis.acceleration.reverseDecelMmS2,
            runtime_.chassis.acceleration.lateralAccelMmS2,
            runtime_.chassis.acceleration.lateralDecelMmS2,
            runtime_.chassis.acceleration.rotationalAccelMradS2,
            runtime_.chassis.acceleration.rotationalDecelMradS2
        };
        for (uint8_t i = 0; i < 8; ++i) {
            data[length++] = static_cast<uint8_t>(values[i]);
            data[length++] = static_cast<uint8_t>(values[i] >> 8);
        }
    } else if (snapshotCursor_ == 10) {
        group = ParameterGroup::ChassisAcceleration; index = 1;
        const uint16_t values[3] = {
            runtime_.chassis.acceleration.zeroCrossingHoldMs,
            runtime_.chassis.translationZeroThresholdMmS,
            runtime_.chassis.rotationZeroThresholdMradS
        };
        for (uint8_t i = 0; i < 3; ++i) {
            data[length++] = static_cast<uint8_t>(values[i]);
            data[length++] = static_cast<uint8_t>(values[i] >> 8);
        }
    } else if (snapshotCursor_ == 11) {
        group = ParameterGroup::Encoder; index = 0;
        const uint16_t values[4] = {
            runtime_.encoder.wheelDiameterMm, runtime_.encoder.countsPerRevolution,
            runtime_.encoder.wheelTrackMm, runtime_.encoder.wheelbaseMm
        };
        for (uint8_t i = 0; i < 4; ++i) {
            data[length++] = static_cast<uint8_t>(values[i]);
            data[length++] = static_cast<uint8_t>(values[i] >> 8);
        }
        data[length++] = static_cast<uint8_t>(runtime_.encoder.semantics);
    } else if (snapshotCursor_ == 12 || snapshotCursor_ == 13) {
        group = ParameterGroup::Encoder;
        index = snapshotCursor_ == 12 ? 1 : 2;
        const int8_t *map = index == 1
            ? runtime_.encoder.channelMap : runtime_.encoder.commandMap;
        const int8_t *signs = index == 1
            ? runtime_.encoder.signs : runtime_.encoder.commandSigns;
        for (uint8_t i = 0; i < 4; ++i) data[length++] = static_cast<uint8_t>(map[i]);
        for (uint8_t i = 0; i < 4; ++i) data[length++] = static_cast<uint8_t>(signs[i]);
    } else if (snapshotCursor_ < 20) {
        group = ParameterGroup::Sensor; index = snapshotCursor_ - 14;
        const int16_t value = runtime_.sensorOffsetMm[index];
        data[0] = static_cast<uint8_t>(value);
        data[1] = static_cast<uint8_t>(value >> 8); length = 2;
    } else if (snapshotCursor_ == 20) {
        group = ParameterGroup::Assist;
        const uint16_t values[3] = {
            runtime_.normalDriveLimitPermille,
            runtime_.cargoDriveLimitPermille,
            runtime_.assistDriveLimitPermille
        };
        for (uint8_t i = 0; i < 3; ++i) {
            data[length++] = static_cast<uint8_t>(values[i]);
            data[length++] = static_cast<uint8_t>(values[i] >> 8);
        }
    } else if (snapshotCursor_ == 21) {
        group = ParameterGroup::ResponseProfile;
        data[0] = static_cast<uint8_t>(runtime_.chassis.activeProfile);
        length = 1;
    } else if (snapshotCursor_ < 25) {
        group = ParameterGroup::ResponseProfile;
        index = snapshotCursor_ - 21;
        const ResponseProfileDefinition &definition =
            runtime_.responseProfiles[snapshotCursor_ - 22];
        const uint16_t values[3] = {
            definition.speedPermille, definition.accelerationPermille,
            definition.decelerationPermille
        };
        for (uint8_t i = 0; i < 3; ++i) {
            data[length++] = static_cast<uint8_t>(values[i]);
            data[length++] = static_cast<uint8_t>(values[i] >> 8);
        }
#if ROBOT_CALIBRATION
    } else {
        group = ParameterGroup::ArmGeometry;
        index = snapshotCursor_ - 25;
        if (index == 0) {
            const uint16_t values[8] = {
                runtime_.arm.firstLinkMm, runtime_.arm.secondLinkMm,
                runtime_.arm.shoulderBaseHeightMm,
                runtime_.arm.gripperLengthOffsetMm,
                runtime_.arm.minimumReachMm, runtime_.arm.maximumReachMm,
                runtime_.arm.minimumHeightMm, runtime_.arm.maximumHeightMm
            };
            for (uint8_t i = 0; i < 8; ++i) {
                data[length++] = static_cast<uint8_t>(values[i]);
                data[length++] = static_cast<uint8_t>(values[i] >> 8);
            }
        } else {
            const uint16_t values[5] = {
                runtime_.arm.cargoClearanceHeightMm, runtime_.arm.presetReachMm,
                runtime_.arm.presetHeightMm, runtime_.arm.stowReachMm,
                runtime_.arm.stowHeightMm
            };
            for (uint8_t i = 0; i < 5; ++i) {
                data[length++] = static_cast<uint8_t>(values[i]);
                data[length++] = static_cast<uint8_t>(values[i] >> 8);
            }
        }
#endif
    }
    payload[offset++] = static_cast<uint8_t>(group);
    payload[offset++] = index;
    putU16(payload, offset, runtime_.revision);
    for (uint8_t i = 0; i < length; ++i) payload[offset++] = data[i];
    communication_.sendFrame(
        hostStream(), MessageType::ParameterSnapshot, snapshotSequence_,
        payload, offset
    );
    if (++snapshotCursor_ >=
#if ROBOT_CALIBRATION
            27
#else
            25
#endif
       ) snapshotActive_ = false;
    return true;
}

void RobotApplication::sendTelemetry(uint32_t nowMs) {
    if (!communication_.transmitIdle()) {
        saturatingIncrement(droppedTelemetry_);
        return;
    }
    if (helloPending_) {
        sendHello();
        helloPending_ = false;
        return;
    }
    if ((telemetryDetailSlot_ & 1U) == 0) {
        sendStatus(nowMs);
    } else if (sendNextParameterSnapshot()) {
        // Parameter snapshots use detail slots so status remains live.
    } else {
        switch ((telemetryDetailSlot_ / 2U) % 6U) {
#if ROBOT_CALIBRATION
            // Chassis intent and open-loop PWM mirrors are meaningless while
            // the profile cannot arm; their slots repeat the status frame.
            case 0: sendStatus(nowMs); break;
#else
            case 0: sendDriveCommand(nowMs); break;
#endif
            case 1: sendDriveFeedback(nowMs); break;
            case 2: sendEncoderTotals(nowMs); break;
            case 3: sendScheduler(); break;
            case 4: sendSensorArm(nowMs); break;
            default:
#if !ROBOT_CALIBRATION
                if (driveBackend_.capabilities().pwmUnit != PwmUnit::Unavailable) {
                    sendOpenLoopPwm();
                    break;
                }
#endif
                sendStatus(nowMs);
                break;
        }
    }
    ++telemetryDetailSlot_;
}

} // namespace robot
