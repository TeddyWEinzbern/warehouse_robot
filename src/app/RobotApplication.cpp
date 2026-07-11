#include "app/RobotApplication.h"

#include "app/PinProfile.h"

namespace robot {

RobotApplication::RobotApplication()
    : bluetooth_(pins::BluetoothRx, pins::BluetoothTx),
      usbCommunication_(false),
#if defined(ROBOT_BACKEND_UART)
      bluetoothCommunication_(true), driveBackend_(Serial),
#else
      bluetoothCommunication_(false), driveBackend_(),
#endif
      chassis_(driveBackend_), activeFrame_({}),
      status_({AssistStage::Idle, false, false, false, 0}),
      lastProcessedSequence_(255) {}

void RobotApplication::begin() {
#if !defined(ROBOT_BACKEND_UART)
    Serial.begin(config::UsbBaud);
#endif
#if !ROBOT_ARM_CALIBRATION
    bluetooth_.begin(config::BluetoothBaud);
#endif
    chassis_.begin();
    arm_.begin();
#if !ROBOT_ARM_CALIBRATION
    sensors_.begin();
#endif
#if defined(ROBOT_BACKEND_L293D)
    Serial.println(F("warehouse_robot l293d_dev ready"));
#elif ROBOT_ARM_CALIBRATION
    Serial.println(F("arm_calibration ready; send J:<0-3>,<0-180>"));
#endif
}

void RobotApplication::update() {
    const uint32_t nowMs = millis();
#if ROBOT_ARM_CALIBRATION
    calibration_.poll(Serial, arm_);
    return;
#elif defined(ROBOT_BACKEND_UART)
    bluetoothCommunication_.poll(bluetooth_, nowMs);
    activeFrame_ = bluetoothCommunication_.latest();
#else
    usbCommunication_.poll(Serial, nowMs);
    bluetoothCommunication_.poll(bluetooth_, nowMs);
    const OperatorControlFrame &usb = usbCommunication_.latest();
    const OperatorControlFrame &bt = bluetoothCommunication_.latest();
    activeFrame_ = bt.valid && (!usb.valid || bt.receivedAtMs > usb.receivedAtMs) ? bt : usb;
#endif

    if (activeFrame_.valid) {
        if (activeFrame_.sequence == lastProcessedSequence_) activeFrame_.pressed = 0;
        else lastProcessedSequence_ = activeFrame_.sequence;
    }

    sensors_.update(nowMs, micros());
    arm_.requestPreset(activeFrame_.pressed);
    const AssistOutput assistOutput = assist_.update(
        activeFrame_, sensors_.snapshot(), arm_.currentTarget(), arm_.calibrated(), nowMs
    );
    if (assistOutput.reachActive) arm_.requestReach(assistOutput.requestedReachMm);
    arm_.update(activeFrame_, nowMs);
    const DriveIntent drive = safety_.arbitrate(
        activeFrame_, assistOutput, arm_.cargoMayBeHeld(), nowMs
    );
    if (drive.maxMagnitude == 0) {
        chassis_.stop();
        chassis_.backend().poll(nowMs);
    } else if (activeFrame_.directWheels) {
        const WheelTargets direct = {
            activeFrame_.wheelFrontLeft, activeFrame_.wheelFrontRight,
            activeFrame_.wheelRearLeft, activeFrame_.wheelRearRight
        };
        chassis_.updateDirect(direct, drive.maxMagnitude, nowMs);
    }
    else chassis_.update(drive, nowMs);

    status_.assistStage = assistOutput.stage;
    status_.cargoMayBeHeld = arm_.cargoMayBeHeld();
    status_.linkAlive = safety_.linkAlive();
    status_.emergencyStopped = safety_.emergencyStopped();
}

} // namespace robot
