#pragma once

#include <Arduino.h>
#include <SoftwareSerial.h>

#include "app/BuildConfig.h"
#include "app/CalibrationConsole.h"
#include "subsystems/ArmSubsystem.h"
#include "subsystems/AssistSubsystem.h"
#include "subsystems/ChassisSubsystem.h"
#include "subsystems/CommunicationSubsystem.h"
#include "subsystems/SafetySupervisor.h"
#include "subsystems/SensorSubsystem.h"

#if defined(ROBOT_BACKEND_L293D)
#include "drivers/L293DDriveBackend.h"
#elif defined(ROBOT_BACKEND_UART)
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
    CommunicationSubsystem usbCommunication_;
    CommunicationSubsystem bluetoothCommunication_;
#if defined(ROBOT_BACKEND_L293D)
    L293DDriveBackend driveBackend_;
#elif defined(ROBOT_BACKEND_UART)
    UartEncoderDriveBackend driveBackend_;
#else
    NullDriveBackend driveBackend_;
#endif
    ChassisSubsystem chassis_;
    ArmSubsystem arm_;
    SensorSubsystem sensors_;
    AssistSubsystem assist_;
    SafetySupervisor safety_;
    CalibrationConsole calibration_;
    OperatorControlFrame activeFrame_;
    RobotStatus status_;
    uint8_t lastProcessedSequence_;
};

} // namespace robot
