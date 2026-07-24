#pragma once

#include <Arduino.h>
#include <NeoSWSerial.h>

#include "app/BuildConfig.h"
#include "core/Scheduler.h"
#include "domain/RuntimeConfig.h"
#include "subsystems/ArmSubsystem.h"
#include "subsystems/AssistSubsystem.h"
#include "subsystems/ChassisSubsystem.h"
#include "subsystems/CommunicationSubsystem.h"
#include "subsystems/SafetySupervisor.h"
#include "subsystems/SensorSubsystem.h"

#if ROBOT_DRIVER_ENABLED
#include "drivers/UartEncoderDriveBackend.h"
#else
#include "drivers/NullDriveBackend.h"
#endif

namespace robot {

class RobotApplication {
  public:
    RobotApplication();
    void begin();
    void update();

  private:
    NeoSWSerial bluetooth_;
    CommunicationSubsystem communication_;
#if ROBOT_DRIVER_ENABLED
    UartEncoderDriveBackend driveBackend_;
#else
    NullDriveBackend driveBackend_;
#endif
    ChassisSubsystem chassis_;
    ArmSubsystem arm_;
    SensorSubsystem sensors_;
    AssistSubsystem assist_;
    SafetySupervisor safety_;
    RuntimeConfig runtime_;
    OperatorControlFrame activeFrame_;
    AssistOutput assistOutput_;
    SchedulerHealth schedulerHealth_;
    PeriodicTask chassisTask_;
    PeriodicTask motorTask_;
    PeriodicTask servoTask_;
    PeriodicDeadline encoderTask_;
    PeriodicDeadline sonarTask_;
    PeriodicDeadline encoderTotalTask_;
    PeriodicDeadline statusTask_;
    uint32_t lastControlReceivedUs_;
    uint32_t missWindowStartedUs_;
    uint16_t motorMissBaseline_;
    uint16_t chassisMissBaseline_;
    uint16_t lastStatusFaults_;
    uint16_t lastStatusWarnings_;
    uint8_t consecutiveMotorLate_;
    uint8_t lastProcessedControlSequence_;
    uint8_t transmitSequence_;
    RobotState previousState_;
    RobotState lastStatusState_;
    bool lastStatusLinkAlive_;
    bool statusPending_;
    bool helloPending_;
    bool platformInitialized_;

    Stream &hostStream();
    RobotProfile profile() const;
    DriveHealth effectiveDriveHealth(uint32_t nowMs) const;
    bool profileCanArm() const;
    bool sonarEnabled() const;
    bool armMotionEnabled() const;
    bool hostTransmitSafe(uint32_t nowUs) const;
    void processHostMessages(uint32_t nowMs);
    void enforceSafetyStop(uint32_t nowMs);
    void updateMotionIntent(uint32_t nowMs);
    void runDueTasks(uint32_t nowMs, uint32_t nowUs);
    void evaluateMissWindow(uint32_t nowUs);
    void updateStatusPending(uint32_t nowMs);
    void serviceHostTransmit(uint32_t nowMs, uint32_t nowUs);
    bool sendHello();
    bool sendCriticalStatus(uint32_t nowMs);
    bool sendCalibrationReport(
        const PendingCalibrationRead &request,
        uint32_t nowMs
    );
};

} // namespace robot
