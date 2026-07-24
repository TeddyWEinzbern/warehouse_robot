#include "app/RobotApplication.h"

#include "app/PinProfile.h"
#include "core/StackWatermark.h"

#if defined(ARDUINO_ARCH_AVR)
#include <avr/io.h>
#endif

namespace robot {
namespace {
void putU16(uint8_t *output, uint8_t &offset, uint16_t value) {
    output[offset++] = static_cast<uint8_t>(value);
    output[offset++] = static_cast<uint8_t>(value >> 8);
}

#if ROBOT_CALIBRATION
void putI16(uint8_t *output, uint8_t &offset, int16_t value) {
    putU16(output, offset, static_cast<uint16_t>(value));
}

void putU32(uint8_t *output, uint8_t &offset, uint32_t value) {
    output[offset++] = static_cast<uint8_t>(value);
    output[offset++] = static_cast<uint8_t>(value >> 8);
    output[offset++] = static_cast<uint8_t>(value >> 16);
    output[offset++] = static_cast<uint8_t>(value >> 24);
}
#endif

} // namespace

RobotApplication::RobotApplication()
    : bluetooth_(pins::BluetoothRx, pins::BluetoothTx), communication_(),
#if ROBOT_DRIVER_ENABLED
      driveBackend_(Serial),
#else
      driveBackend_(),
#endif
      chassis_(driveBackend_), runtime_(RuntimeConfig::defaults()),
      activeFrame_({}),
      assistOutput_({{0, 0, 0, 0, IntentSource::Safety}, 0.0F,
                     AssistStage::Idle, false, false}),
      lastControlReceivedUs_(0), missWindowStartedUs_(0),
      motorMissBaseline_(0),
      chassisMissBaseline_(0),
      lastStatusFaults_(0xFFFFU), lastStatusWarnings_(0xFFFFU),
      consecutiveMotorLate_(0), lastProcessedControlSequence_(0xFF),
      transmitSequence_(0), previousState_(RobotState::Boot),
      lastStatusState_(RobotState::Fault), lastStatusLinkAlive_(true),
      statusPending_(ROBOT_CALIBRATION == 0), helloPending_(false),
      platformInitialized_(false) {}

Stream &RobotApplication::hostStream() {
    // D0/D1 belongs exclusively to the motor board. The HC-06 host link uses
    // NeoSWSerial on A5/A4 in production and calibration images.
    return bluetooth_;
}

RobotProfile RobotApplication::profile() const {
#if ROBOT_CALIBRATION
    return RobotProfile::Calibration;
#elif ROBOT_DRIVER_CONTROL_OPEN
    return RobotProfile::UartOpenLoopRobot;
#else
    return RobotProfile::UartClosedLoopRobot;
#endif
}

DriveHealth RobotApplication::effectiveDriveHealth(uint32_t nowMs) const {
    if (!config::DriveEnabled) return {0, 0, true, true, true};
    return driveBackend_.health(nowMs);
}

bool RobotApplication::profileCanArm() const {
#if ROBOT_CALIBRATION
    return false;
#else
    const bool hasMotionFeature = config::DriveEnabled || config::ArmEnabled;
    return hasMotionFeature &&
           (!config::DriveEnabled || config::DriveCalibrated) &&
           (!config::ArmEnabled || config::ArmCalibrated);
#endif
}

bool RobotApplication::sonarEnabled() const {
    return config::SensorEnabled;
}

bool RobotApplication::armMotionEnabled() const {
    return config::ArmEnabled && arm_.calibrated();
}

void RobotApplication::begin() {
    bluetooth_.begin(static_cast<uint16_t>(config::BluetoothBaud));
    if (config::DriveEnabled) chassis_.begin(runtime_);
    if (config::ArmEnabled) arm_.begin(runtime_);
    if (sonarEnabled()) sensors_.begin();

    const uint32_t nowUs = micros();
    chassisTask_.start(nowUs, config::ChassisRampPeriodUs, 0);
    motorTask_.start(nowUs, config::MotorCommandPeriodUs, 0);
    encoderTask_.start(nowUs, config::EncoderQueryPeriodUs, 5000UL);
    servoTask_.start(nowUs, config::ServoPeriodUs, 10000UL);
    sonarTask_.start(nowUs, config::SonarGroupPeriodUs, 15000UL);
    encoderTotalTask_.start(
        nowUs, config::EncoderTotalPeriodUs, 250000UL
    );
    statusTask_.start(
        nowUs, config::CriticalStatusPeriodUs,
        config::CriticalStatusPeriodUs
    );
    schedulerHealth_.begin(nowUs);
    missWindowStartedUs_ = nowUs;
    platformInitialized_ = true;
#if ROBOT_CALIBRATION
    beginStackWatermark();
#endif
}

void RobotApplication::processHostMessages(uint32_t nowMs) {
    if (!communication_.transmitIdle()) return;

    PendingArmMove armMove = {};
    if (communication_.takeArmMove(armMove)) {
#if ROBOT_CALIBRATION
        if (!config::ArmEnabled ||
            safety_.state() != RobotState::Disarmed ||
            armMove.joint >= 4 || armMove.degrees > 180) {
            communication_.sendNack(
                armMove.sequence, MessageType::CalibrationArmMove,
                NackReason::InvalidState
            );
        } else if (!arm_.setCalibrationJoint(
                       armMove.joint, armMove.degrees, runtime_
                   )) {
            communication_.sendNack(
                armMove.sequence, MessageType::CalibrationArmMove,
                NackReason::ValidationFailed
            );
        } else {
            communication_.sendAck(
                armMove.sequence, MessageType::CalibrationArmMove
            );
        }
#else
        communication_.sendNack(
            armMove.sequence, MessageType::CalibrationArmMove,
            NackReason::InvalidState
        );
#endif
        return;
    }

    PendingJointReference reference = {};
    if (communication_.takeJointReference(reference)) {
#if ROBOT_CALIBRATION
        if (!config::ArmEnabled ||
            safety_.state() != RobotState::Disarmed) {
            communication_.sendNack(
                reference.sequence,
                MessageType::CalibrationSetJointReference,
                NackReason::InvalidState
            );
        } else if (!runtime_.setCalibrationServoReference(
                       reference.joint, reference.lowerDegrees,
                       reference.upperDegrees, reference.centerOffsetDegrees,
                       reference.direction
                   )) {
            communication_.sendNack(
                reference.sequence,
                MessageType::CalibrationSetJointReference,
                NackReason::ValidationFailed
            );
        } else {
            communication_.sendAck(
                reference.sequence,
                MessageType::CalibrationSetJointReference
            );
        }
#else
        communication_.sendNack(
            reference.sequence,
            MessageType::CalibrationSetJointReference,
            NackReason::InvalidState
        );
#endif
        return;
    }

    PendingDriveCalibration spin = {};
    if (communication_.takeDriveCalibration(spin)) {
#if ROBOT_CALIBRATION && ROBOT_DRIVER_ENABLED
        const int32_t magnitude = spin.value < 0
            ? -static_cast<int32_t>(spin.value)
            : static_cast<int32_t>(spin.value);
        const int32_t limit = spin.mode == 0
            ? config::CalibrationSpinLimitPercent
            : config::CalibrationWheelLimitMmS;
        if (!config::DriveEnabled ||
            safety_.state() != RobotState::Disarmed) {
            communication_.sendNack(
                spin.sequence, MessageType::CalibrationDriveSpin,
                NackReason::InvalidState
            );
        } else if (spin.mode > 1 || spin.channel > 3 ||
                   magnitude > limit ||
                   spin.durationMs >
                       config::CalibrationSpinMaxDurationMs) {
            communication_.sendNack(
                spin.sequence, MessageType::CalibrationDriveSpin,
                NackReason::ValidationFailed
            );
        } else if (!driveBackend_.startCalibrationSpin(
                       spin.mode, spin.channel, spin.value,
                       spin.durationMs, nowMs
                   )) {
            communication_.sendNack(
                spin.sequence, MessageType::CalibrationDriveSpin,
                NackReason::InvalidState
            );
        } else {
            communication_.sendAck(
                spin.sequence, MessageType::CalibrationDriveSpin
            );
        }
#else
        communication_.sendNack(
            spin.sequence, MessageType::CalibrationDriveSpin,
            NackReason::InvalidState
        );
#endif
        return;
    }

    PendingCalibrationRead request = {};
    if (communication_.takeCalibrationRead(request)) {
#if ROBOT_CALIBRATION
        if (safety_.state() != RobotState::Disarmed ||
            !sendCalibrationReport(request, nowMs)) {
            communication_.sendNack(
                request.sequence, request.type,
                NackReason::InvalidState
            );
        }
#else
        communication_.sendNack(
            request.sequence, request.type, NackReason::InvalidState
        );
#endif
    }
}

void RobotApplication::enforceSafetyStop(uint32_t nowMs) {
    if (!safety_.takeImmediateStop()) return;
    if (config::DriveEnabled) chassis_.forceZero(nowMs);
    assist_.cancel();
    if (config::ArmEnabled) arm_.holdLastCommanded();
}

void RobotApplication::updateMotionIntent(uint32_t nowMs) {
#if ROBOT_CALIBRATION
    (void)nowMs;
#else
    if (!safety_.armed()) {
        assist_.cancel();
        if (config::DriveEnabled) {
            chassis_.setDesired(
                {0, 0, 0, 0, IntentSource::Safety}, runtime_
            );
        }
        return;
    }

    if (config::ArmEnabled)
        arm_.requestPreset(activeFrame_.pressed, runtime_);

    if (config::SensorEnabled) {
        assistOutput_ = assist_.update(
            activeFrame_, sensors_.snapshot(), arm_.currentTarget(),
            armMotionEnabled(), nowMs, runtime_
        );
    } else {
        assist_.cancel();
        assistOutput_ = {
            {0, 0, 0, 0, IntentSource::Safety}, 0.0F,
            AssistStage::Idle, false, false
        };
    }

    if (assistOutput_.reachActive && armMotionEnabled())
        arm_.requestReach(assistOutput_.requestedReachMm, runtime_);
    if (config::DriveEnabled) {
        const DriveIntent intent = safety_.arbitrate(
            activeFrame_, assistOutput_,
            config::ArmEnabled && arm_.cargoMayBeHeld()
        );
        chassis_.setDesired(intent, runtime_);
    }
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
    if (config::DriveEnabled && safety_.armed() &&
        (motorMisses >= 3 || chassisMisses >= 5))
        safety_.latchFault(FaultSchedulerOverrun);
    motorMissBaseline_ = motorTask_.stats().missed;
    chassisMissBaseline_ = chassisTask_.stats().missed;
    missWindowStartedUs_ = nowUs;
}

void RobotApplication::runDueTasks(uint32_t nowMs, uint32_t nowUs) {
    if (chassisTask_.due(nowUs)) {
        uint32_t elapsedUs = chassisTask_.elapsedUs(
            nowUs, 0xFFFFFFFFUL
        );
        if (elapsedUs > config::MaxMotionDtUs) {
            elapsedUs = config::MaxMotionDtUs;
        }
#if !ROBOT_CALIBRATION
        if (config::DriveEnabled && safety_.armed())
            chassis_.trajectoryTick(nowUs, elapsedUs, runtime_);
#endif
        if (config::DriveEnabled && safety_.armed() &&
            chassisTask_.stats().consecutiveMisses >= 3)
            safety_.latchFault(FaultSchedulerOverrun);
        enforceSafetyStop(nowMs);
    }

    if (motorTask_.due(nowUs)) {
        if (motorTask_.lastLatenessUs() >
                config::MotorLateThresholdUs) {
            if (consecutiveMotorLate_ != 255) ++consecutiveMotorLate_;
        } else {
            consecutiveMotorLate_ = 0;
        }
        if (config::DriveEnabled && safety_.armed() &&
            consecutiveMotorLate_ >= 2)
            safety_.latchFault(FaultSchedulerOverrun);
        enforceSafetyStop(nowMs);
#if !ROBOT_CALIBRATION
        if (config::DriveEnabled && safety_.armed())
            chassis_.motorTick(nowMs, true, runtime_);
#endif
    }

    if (encoderTask_.due(nowUs) && config::DriveEnabled &&
        communication_.transmitIdle())
        driveBackend_.onEncoderDeadline(nowMs, runtime_);

    if (servoTask_.due(nowUs)) {
        uint32_t elapsedUs = servoTask_.elapsedUs(
            nowUs, 0xFFFFFFFFUL
        );
        if (elapsedUs > config::MaxMotionDtUs) {
            elapsedUs = config::MaxMotionDtUs;
        }
        if (config::ArmEnabled) {
#if ROBOT_CALIBRATION
            if (safety_.state() == RobotState::Disarmed)
                arm_.calibrationTick(elapsedUs, runtime_);
#else
            if (safety_.armed() && armMotionEnabled())
                arm_.update(activeFrame_, elapsedUs, runtime_);
#endif
            if (safety_.armed() &&
                servoTask_.stats().consecutiveMisses >= 3)
                safety_.latchFault(FaultSchedulerOverrun);
            if (arm_.faulted()) safety_.latchFault(FaultArmTarget);
        }
        enforceSafetyStop(nowMs);
    }

    if (sonarTask_.due(nowUs) && sonarEnabled())
        sensors_.startNextGroup(nowMs, nowUs);

    if (encoderTotalTask_.due(nowUs) && config::DriveEnabled &&
        communication_.transmitIdle())
        driveBackend_.onEncoderTotalDeadline(nowMs);

#if !ROBOT_CALIBRATION
    if (statusTask_.due(nowUs)) statusPending_ = true;
#endif

    evaluateMissWindow(nowUs);
    enforceSafetyStop(nowMs);
    if (config::DriveEnabled) driveBackend_.service(nowMs, runtime_);
}

bool RobotApplication::hostTransmitSafe(uint32_t nowUs) const {
    // Never mask NeoSWSerial receive interrupts while a host frame is only
    // partially parsed. Calibration is strict request/reply and has no
    // unsolicited periodic status; production adds the fixed 30 Hz window.
    if (!communication_.receiveIdle()) return false;
    // NeoSWSerial masks interrupts while sending a byte. Do not blind the
    // polling echo-capture path between a sonar trigger and completion.
    if (sonarEnabled() && sensors_.capturing()) return false;
#if !ROBOT_CALIBRATION
    // The host starts one fixed 30 Hz control frame per period. Wait until a
    // complete frame has arrived, then stop transmitting before the next
    // frame can begin. This prevents NeoSWSerial TX interrupt masking from
    // corrupting the higher-priority control/E-stop receive lane.
    if (!activeFrame_.valid) return false;
    const uint32_t sinceControlUs = nowUs - lastControlReceivedUs_;
    if (sinceControlUs < config::HostTransmitStartDelayUs ||
        sinceControlUs >= config::HostTransmitWindowEndUs)
        return false;
#endif
    if (config::DriveEnabled) {
        if (driveBackend_.outstandingQuery() != 0 ||
            Serial.available() != 0 ||
            motorTask_.untilDeadlineUs(nowUs) < 2000UL ||
            encoderTask_.untilDeadlineUs(nowUs) < 2000UL)
            return false;
    }
#if defined(ARDUINO_ARCH_AVR)
    if (config::ArmEnabled && arm_.servoTimingActive()) {
        // Servo writes for four channels occupy at most the first 9.6 ms of
        // Timer1's 20 ms refresh. NeoSWSerial masks interrupts for most of one
        // transmitted byte, so transmit only in the quiet part of the cycle.
        const uint16_t servoPhaseUs = static_cast<uint16_t>(TCNT1 / 2U);
        if (servoPhaseUs < 10000U || servoPhaseUs > 18000U)
            return false;
    }
#else
    (void)nowUs;
#endif
    return true;
}

void RobotApplication::updateStatusPending(uint32_t nowMs) {
    const DriveHealth drive = effectiveDriveHealth(nowMs);
    const uint16_t faults = static_cast<uint16_t>(
        safety_.faults() | drive.faults
    );
    const uint16_t warnings = static_cast<uint16_t>(
        drive.warnings |
        (config::ArmEnabled && arm_.targetLimited()
             ? WarningArmTargetLimited : 0U)
    );
    if (safety_.state() != lastStatusState_ ||
        faults != lastStatusFaults_ ||
        warnings != lastStatusWarnings_ ||
        safety_.linkAlive() != lastStatusLinkAlive_)
        statusPending_ = true;
}

bool RobotApplication::sendHello() {
    uint8_t driverMode = 0;
#if ROBOT_DRIVER_ENABLED
#if ROBOT_DRIVER_CONTROL_OPEN
    driverMode = 1;
#else
    driverMode = 2;
#endif
#endif
    const uint8_t payload[8] = {
        static_cast<uint8_t>(profile()),
        static_cast<uint8_t>(config::ArmEnabled),
        static_cast<uint8_t>(config::DriveEnabled),
        static_cast<uint8_t>(config::SensorEnabled),
        driverMode,
        static_cast<uint8_t>(
            config::ArmEnabled && config::ArmCalibrated
        ),
        static_cast<uint8_t>(
            config::DriveEnabled && config::DriveCalibrated
        ),
        static_cast<uint8_t>(config::BluetoothBaud / 1200UL),
    };
    return communication_.sendFrame(
        MessageType::HelloResponse, transmitSequence_++,
        payload, sizeof(payload)
    );
}

bool RobotApplication::sendCriticalStatus(uint32_t nowMs) {
    const DriveHealth drive = effectiveDriveHealth(nowMs);
    const uint16_t faults = static_cast<uint16_t>(
        safety_.faults() | drive.faults
    );
    const uint16_t warnings = static_cast<uint16_t>(
        drive.warnings |
        (config::ArmEnabled && arm_.targetLimited()
             ? WarningArmTargetLimited : 0U)
    );
    uint8_t payload[7];
    uint8_t offset = 0;
    payload[offset++] = static_cast<uint8_t>(safety_.state());
    putU16(payload, offset, faults);
    putU16(payload, offset, warnings);
    payload[offset++] = lastProcessedControlSequence_;
    payload[offset++] = static_cast<uint8_t>(safety_.linkAlive());
    if (!communication_.sendFrame(
            MessageType::CriticalStatus, transmitSequence_++,
            payload, offset
        ))
        return false;
    lastStatusState_ = safety_.state();
    lastStatusFaults_ = faults;
    lastStatusWarnings_ = warnings;
    lastStatusLinkAlive_ = safety_.linkAlive();
    return true;
}

bool RobotApplication::sendCalibrationReport(
    const PendingCalibrationRead &request, uint32_t nowMs
) {
#if ROBOT_CALIBRATION
    if (request.type == MessageType::CalibrationReadArm) {
        if (!config::ArmEnabled) return false;
        uint8_t payload[5] = {
            static_cast<uint8_t>(CalibrationReportKind::Arm),
            0, 0, 0, 0
        };
        const uint8_t *targets = arm_.lastCommandedDegrees();
        for (uint8_t joint = 0; joint < 4; ++joint)
            payload[joint + 1] = targets[joint];
        return communication_.sendFrame(
            MessageType::CalibrationReport, request.sequence,
            payload, sizeof(payload)
        );
    }

    if (request.type == MessageType::CalibrationReadDrive) {
        if (!config::DriveEnabled) return false;
        const DriveFeedback &feedback = driveBackend_.feedback();
        if (request.page == 0) {
            uint8_t payload[26];
            uint8_t offset = 0;
            payload[offset++] = static_cast<uint8_t>(
                CalibrationReportKind::DriveCounts
            );
            for (uint8_t channel = 0; channel < 4; ++channel)
                putI16(payload, offset, feedback.rawIncrement[channel]);
            for (uint8_t channel = 0; channel < 4; ++channel)
                putU32(
                    payload, offset,
                    static_cast<uint32_t>(feedback.total[channel])
                );
            const bool incrementFresh =
                feedback.incrementUpdatedAtMs != 0 &&
                nowMs - feedback.incrementUpdatedAtMs <=
                    config::FeedbackStaleMs;
            payload[offset++] = incrementFresh
                ? static_cast<uint8_t>(
                      feedback.totalValidMask &
                      feedback.encoderValidMask
                  )
                : 0;
            return communication_.sendFrame(
                MessageType::CalibrationReport, request.sequence,
                payload, offset
            );
        }
        uint8_t payload[10];
        uint8_t offset = 0;
        payload[offset++] = static_cast<uint8_t>(
            CalibrationReportKind::DriveSpeed
        );
        for (uint8_t wheel = 0; wheel < 4; ++wheel)
            putI16(payload, offset, feedback.measuredMmS[wheel]);
        payload[offset++] =
            nowMs - feedback.incrementUpdatedAtMs <=
                    config::FeedbackStaleMs
                ? feedback.encoderValidMask : 0;
        return communication_.sendFrame(
            MessageType::CalibrationReport, request.sequence,
            payload, offset
        );
    }

    if (request.type == MessageType::CalibrationReadSensor) {
        if (!config::SensorEnabled) return false;
        uint8_t payload[14];
        uint8_t offset = 0;
        uint8_t validMask = 0;
        payload[offset++] = static_cast<uint8_t>(
            CalibrationReportKind::Sensor
        );
        for (uint8_t direction = 0; direction < 3; ++direction) {
            const DistancePair &pair =
                sensors_.snapshot().directions[direction];
            const DistanceReading readings[2] = {
                pair.first, pair.second
            };
            for (uint8_t slot = 0; slot < 2; ++slot) {
                const uint8_t bit = static_cast<uint8_t>(
                    direction * 2U + slot
                );
                if (readings[slot].valid &&
                    nowMs - readings[slot].updatedAtMs <=
                        config::SensorStaleMs)
                    validMask |= static_cast<uint8_t>(1U << bit);
                putU16(payload, offset, readings[slot].millimetres);
            }
        }
        payload[offset++] = validMask;
        return communication_.sendFrame(
            MessageType::CalibrationReport, request.sequence,
            payload, offset
        );
    }

    if (request.type == MessageType::CalibrationReadSystem) {
        uint8_t payload[3];
        uint8_t offset = 0;
        payload[offset++] = static_cast<uint8_t>(
            CalibrationReportKind::System
        );
        putU16(payload, offset, minimumFreeStackBytes());
        return communication_.sendFrame(
            MessageType::CalibrationReport, request.sequence,
            payload, offset
        );
    }
#else
    (void)request;
    (void)nowMs;
#endif
    return false;
}

void RobotApplication::serviceHostTransmit(
    uint32_t nowMs, uint32_t nowUs
) {
    if (communication_.transmitIdle()) {
        if (helloPending_) {
            if (sendHello()) helloPending_ = false;
        } else if (statusPending_) {
            if (sendCriticalStatus(nowMs)) statusPending_ = false;
        }
    }
    if (hostTransmitSafe(nowUs))
        communication_.pumpTransmit(hostStream(), 1);
}

void RobotApplication::update() {
    const uint32_t nowUs = micros();
    const uint32_t nowMs = millis();
    if (schedulerHealth_.observeLoop(nowUs, safety_.armed()))
        safety_.latchFault(FaultSchedulerOverrun);

    const ControlRequests noRequests = {0};
    safety_.update(
        activeFrame_, noRequests, effectiveDriveHealth(nowMs),
        platformInitialized_, profileCanArm(), nowMs
    );
    enforceSafetyStop(nowMs);

    if (config::DriveEnabled)
        driveBackend_.pollReceive(nowMs, runtime_);
    communication_.poll(hostStream(), nowMs);
    if (sonarEnabled()) sensors_.poll(nowMs, nowUs);

    const OperatorControlFrame &received = communication_.latest();
    if (received.valid) {
        if (!activeFrame_.valid ||
            received.receivedAtMs != activeFrame_.receivedAtMs)
            lastControlReceivedUs_ = micros();
        activeFrame_ = received;
        if (activeFrame_.sequence == lastProcessedControlSequence_)
            activeFrame_.pressed = 0;
        else
            lastProcessedControlSequence_ = activeFrame_.sequence;
    }

    const ControlRequests requests = communication_.takeRequests();
    if ((requests.flags & RequestHello) != 0) helloPending_ = true;
    if (config::ArmEnabled && arm_.faulted())
        safety_.latchFault(FaultArmTarget);
    safety_.update(
        activeFrame_, requests, effectiveDriveHealth(nowMs),
        platformInitialized_, profileCanArm(), nowMs
    );
    if (safety_.takeClearFaultAccepted()) {
        if (config::DriveEnabled) driveBackend_.clearFaults();
        if (config::ArmEnabled) arm_.clearFault();
    }
    enforceSafetyStop(nowMs);
    processHostMessages(nowMs);

    if (previousState_ != safety_.state()) {
        if (safety_.state() == RobotState::Armed &&
            config::ArmEnabled)
            arm_.releaseHold();
        previousState_ = safety_.state();
    }

    if (config::DriveEnabled) driveBackend_.service(nowMs, runtime_);
    updateMotionIntent(nowMs);
    activeFrame_.pressed = 0;
    runDueTasks(nowMs, nowUs);
#if !ROBOT_CALIBRATION
    updateStatusPending(nowMs);
#endif
    serviceHostTransmit(millis(), micros());
}

} // namespace robot
