#pragma once

#include <Arduino.h>
#include <SoftwareSerial.h>

#include "app/BuildConfig.h"
#include "core/Scheduler.h"
#include "domain/RuntimeConfig.h"
#include "subsystems/ArmSubsystem.h"
#include "subsystems/AssistSubsystem.h"
#include "subsystems/ChassisSubsystem.h"
#include "subsystems/CommunicationSubsystem.h"
#include "subsystems/SafetySupervisor.h"
#include "subsystems/SensorSubsystem.h"

#if defined(ROBOT_BACKEND_UART)
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
    SoftwareSerial bluetooth_;
    CommunicationSubsystem communication_;
#if defined(ROBOT_BACKEND_UART)
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
    RobotStatus status_;
    SchedulerHealth schedulerHealth_;
    PeriodicTask chassisTask_;
    PeriodicTask motorTask_;
    PeriodicTask encoderTask_;
    PeriodicTask servoTask_;
    PeriodicTask sonarTask_;
    PeriodicTask encoderTotalTask_;
    PeriodicTask batteryTask_;
    PeriodicTask telemetryTask_;
    uint32_t missWindowStartedUs_;
    uint16_t motorMissBaseline_;
    uint16_t chassisMissBaseline_;
    uint16_t droppedTelemetry_;
    uint16_t motionDtClamps_;
    uint8_t consecutiveMotorLate_;
    uint8_t lastProcessedControlSequence_;
    uint8_t telemetrySequence_;
    uint8_t telemetryDetailSlot_;
    uint8_t snapshotCursor_;
    uint8_t snapshotSequence_;
    bool snapshotActive_;
    bool helloPending_;
    bool platformInitialized_;
    RobotState previousState_;

    Stream &hostStream();
    RobotProfile profile() const;
    bool profileCanArm() const;
    bool sonarEnabled() const;
    bool armMotionEnabled() const;
    void processHostMessages(uint32_t nowMs, const ControlRequests &requests);
    void enforceSafetyStop(uint32_t nowMs);
    void updateMotionIntent(uint32_t nowMs);
    void runDueTasks(uint32_t nowMs, uint32_t nowUs);
    void evaluateMissWindow(uint32_t nowUs);
    void sendTelemetry(uint32_t nowMs);
    void sendHello();
    void sendStatus(uint32_t nowMs);
    void sendDriveCommand(uint32_t nowMs);
    void sendDriveFeedback(uint32_t nowMs);
    void sendEncoderTotals(uint32_t nowMs);
    void sendScheduler();
    void sendSensorArm(uint32_t nowMs);
    void sendOpenLoopPwm();
    bool sendNextParameterSnapshot();
};

} // namespace robot
