#include "RobotController.h"

RobotController::RobotController()
    : bluetoothSerial_(Config::BLUETOOTH_RX_PIN, Config::BLUETOOTH_TX_PIN),
      mode_(RobotMode::Manual), lastCommandAt_(0), lastDebugAt_(0),
      usbStats_({0, 0, 0}), bluetoothStats_({0, 0, 0}),
      lastCommandSource_("none"), lastCommandKind_('-'),
      lastDriveCommand_({0, 0, 0}) {
}

void RobotController::begin() {
    Serial.begin(Config::USB_BAUD);
    if (readsBluetoothCommands()) {
        bluetoothSerial_.begin(Config::BLUETOOTH_BAUD);
    }

    motors_.begin();
    motors_.stop();
    lastCommandAt_ = millis();
    printStartupBanner();
}

void RobotController::update() {
    if (readsUsbCommands()) {
        processStream(Serial, usbReader_, "USB", usbStats_);
    }
    if (readsBluetoothCommands()) {
        processStream(
            bluetoothSerial_, bluetoothReader_, "Bluetooth", bluetoothStats_
        );
    }

    if (!motors_.isStopped() &&
        millis() - lastCommandAt_ > Config::COMMAND_TIMEOUT_MS) {
        motors_.stop();
    }

    printDebugStatus();
}

bool RobotController::readsUsbCommands() const {
    return Config::ACTIVE_COMMAND_INTERFACE == Config::CommandInterface::Usb ||
           Config::ACTIVE_COMMAND_INTERFACE == Config::CommandInterface::Both;
}

bool RobotController::readsBluetoothCommands() const {
    return Config::ACTIVE_COMMAND_INTERFACE ==
               Config::CommandInterface::Bluetooth ||
           Config::ACTIVE_COMMAND_INTERFACE == Config::CommandInterface::Both;
}

void RobotController::processStream(
    Stream &stream,
    SerialProtocol::CommandReader &reader,
    const char *sourceName,
    CommandStats &stats
) {
    while (stream.available() > 0) {
        RobotCommand rawCommand = {};
        unsigned long bytesRead = 0;
        const SerialProtocol::ReadResult result =
            reader.read(stream, rawCommand, bytesRead);
        stats.bytesReceived += bytesRead;

        if (result == SerialProtocol::ReadResult::ValidCommand) {
            const RobotCommand command = inputMapper_.map(rawCommand);
            if (handleCommand(command, sourceName)) {
                stats.validCommands++;
            } else {
                stats.ignoredCommands++;
            }
        } else if (result == SerialProtocol::ReadResult::InvalidCommand) {
            stats.ignoredCommands++;
            Serial.print(F("Ignored "));
            Serial.print(sourceName);
            Serial.print(F(" command: "));
            Serial.println(reader.lastLine());
        } else if (result == SerialProtocol::ReadResult::Overflow) {
            stats.ignoredCommands++;
        }
    }
}

bool RobotController::handleCommand(
    const RobotCommand &command, const char *sourceName
) {
    if (mode_ != RobotMode::Manual) {
        return false;
    }

    if (command.type == RobotCommandType::Drive) {
        motors_.drive(
            command.drive.forward, command.drive.turn, command.drive.strafe
        );
        lastCommandKind_ = 'C';
        lastCommandSource_ = sourceName;
        lastDriveCommand_ = command.drive;
        lastCommandAt_ = millis();
        return true;
    }

    if (command.type == RobotCommandType::WheelSpeeds) {
        motors_.setWheelSpeeds(
            command.wheels.frontLeft,
            command.wheels.frontRight,
            command.wheels.rearLeft,
            command.wheels.rearRight
        );
        lastCommandKind_ = 'W';
        lastCommandSource_ = sourceName;
        lastDriveCommand_ = {0, 0, 0};
        lastCommandAt_ = millis();
        return true;
    }

    if (command.type == RobotCommandType::SingleMotor) {
        motors_.setSingleMotor(
            command.singleMotor.motorIndex, command.singleMotor.speed
        );
        lastCommandKind_ = 'M';
        lastCommandSource_ = sourceName;
        lastDriveCommand_.forward = command.singleMotor.motorIndex;
        lastDriveCommand_.turn = command.singleMotor.speed;
        lastDriveCommand_.strafe = 0;
        lastCommandAt_ = millis();
        return true;
    }

    return false;
}

void RobotController::printStartupBanner() const {
    Serial.println(F("L293D motor bridge ready."));
    Serial.println(F("Motor shield: L293D V1-compatible M1/M2/M3/M4"));
    Serial.print(F("Drive mixing: "));
    Serial.println(
        Config::ACTIVE_DRIVE_MIXING == Config::DriveMixing::Mecanum
            ? F("Mecanum")
            : F("Tank")
    );
    Serial.println(
        F("Bluetooth serial: Arduino RX=A5 <- module TX, Arduino TX=A4 -> "
          "module RX")
    );
}

void RobotController::printDebugStatus() {
    if (!Config::DEBUG_SERIAL ||
        millis() - lastDebugAt_ < Config::DEBUG_INTERVAL_MS) {
        return;
    }

    lastDebugAt_ = millis();
    const WheelSpeedCommand &wheels = motors_.lastWheelSpeeds();
    Serial.print(F("DBG usb bytes="));
    Serial.print(usbStats_.bytesReceived);
    Serial.print(F(" ok="));
    Serial.print(usbStats_.validCommands);
    Serial.print(F(" bad="));
    Serial.print(usbStats_.ignoredCommands);
    Serial.print(F(" | bt bytes="));
    Serial.print(bluetoothStats_.bytesReceived);
    Serial.print(F(" ok="));
    Serial.print(bluetoothStats_.validCommands);
    Serial.print(F(" bad="));
    Serial.print(bluetoothStats_.ignoredCommands);
    Serial.print(F(" | last "));
    Serial.print(lastCommandSource_);
    Serial.print(F(" "));
    Serial.print(lastCommandKind_);
    Serial.print(F(":"));
    Serial.print(lastDriveCommand_.forward);
    Serial.print(F(","));
    Serial.print(lastDriveCommand_.turn);
    Serial.print(F(","));
    Serial.print(lastDriveCommand_.strafe);
    Serial.print(F(" wheels FL/FR/RL/RR="));
    Serial.print(wheels.frontLeft);
    Serial.print(F("/"));
    Serial.print(wheels.frontRight);
    Serial.print(F("/"));
    Serial.print(wheels.rearLeft);
    Serial.print(F("/"));
    Serial.print(wheels.rearRight);
    Serial.print(F(" stopped="));
    Serial.println(motors_.isStopped() ? F("yes") : F("no"));
}
