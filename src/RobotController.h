#pragma once

#include <Arduino.h>
#include <SoftwareSerial.h>

#include "InputMapper.h"
#include "MotorController.h"
#include "SerialProtocol.h"
#include "State.h"

class RobotController {
  public:
    RobotController();

    void begin();
    void update();

  private:
    SoftwareSerial bluetoothSerial_;
    SerialProtocol::CommandReader usbReader_;
    SerialProtocol::CommandReader bluetoothReader_;
    InputMapper inputMapper_;
    MotorController motors_;
    RobotMode mode_;
    unsigned long lastCommandAt_;
    unsigned long lastDebugAt_;
    CommandStats usbStats_;
    CommandStats bluetoothStats_;
    const char *lastCommandSource_;
    char lastCommandKind_;
    DriveCommand lastDriveCommand_;

    bool readsUsbCommands() const;
    bool readsBluetoothCommands() const;
    void processStream(
        Stream &stream,
        SerialProtocol::CommandReader &reader,
        const char *sourceName,
        CommandStats &stats
    );
    bool handleCommand(const RobotCommand &command, const char *sourceName);
    void printStartupBanner() const;
    void printDebugStatus();
};
