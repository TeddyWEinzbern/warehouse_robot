#include "drivers/UartEncoderDriveBackend.h"

#include "app/BuildConfig.h"

#include <stdlib.h>
#include <string.h>

namespace robot {

UartEncoderDriveBackend::UartEncoderDriveBackend(HardwareSerial &serial)
    : serial_(serial), feedback_({{0, 0, 0, 0}, 0, 0, false, false}),
      receiveLength_(0), lastEncoderQueryMs_(0), lastBatteryQueryMs_(0) {}

void UartEncoderDriveBackend::begin() {
    serial_.begin(config::MotorBoardBaud);
    serial_.print(F("$MOTOR_4CH_SET:0!"));
    serial_.print(F("$MOTOR_4CH_SET_ENCPDER_POLARITY:0!"));
    stop();
}

void UartEncoderDriveBackend::sendSpeeds(const WheelTargets &targets) {
    serial_.print(F("$Car:"));
    serial_.print(targets.frontLeft / 1000.0F, 3); serial_.print(',');
    serial_.print(targets.frontRight / 1000.0F, 3); serial_.print(',');
    serial_.print(targets.rearLeft / 1000.0F, 3); serial_.print(',');
    serial_.print(targets.rearRight / 1000.0F, 3); serial_.print('!');
}

void UartEncoderDriveBackend::setWheelTargets(const WheelTargets &targets) { sendSpeeds(targets); }
void UartEncoderDriveBackend::stop() { sendSpeeds({0, 0, 0, 0}); }

bool UartEncoderDriveBackend::parseFourLongs(const char *cursor, int32_t *values) {
    for (uint8_t index = 0; index < 4; ++index) {
        char *end = 0;
        values[index] = strtol(cursor, &end, 10);
        if (end == cursor || (index < 3 && *end != ',')) return false;
        cursor = index < 3 ? end + 1 : end;
    }
    return *cursor == '\0';
}

void UartEncoderDriveBackend::finishMessage(uint32_t nowMs) {
    receive_[receiveLength_] = '\0';
    const char encoderPrefix[] = "$MOTOR_4CH_Encoder_20ms:";
    const char batteryPrefix[] = "$MOTOR_4CH_Battery:";
    if (strncmp(receive_, encoderPrefix, sizeof(encoderPrefix) - 1) == 0) {
        if (parseFourLongs(receive_ + sizeof(encoderPrefix) - 1, feedback_.encoder)) {
            feedback_.encoderValid = true;
            feedback_.updatedAtMs = nowMs;
        }
    } else if (strncmp(receive_, batteryPrefix, sizeof(batteryPrefix) - 1) == 0) {
        char *end = 0;
        const long value = strtol(receive_ + sizeof(batteryPrefix) - 1, &end, 10);
        if (end != receive_ + sizeof(batteryPrefix) - 1 && *end == '\0' && value >= 0 && value <= 65535) {
            feedback_.batteryMv = static_cast<uint16_t>(value);
            feedback_.batteryValid = true;
            feedback_.updatedAtMs = nowMs;
        }
    }
    receiveLength_ = 0;
}

void UartEncoderDriveBackend::poll(uint32_t nowMs) {
    while (serial_.available() > 0) {
        const char value = static_cast<char>(serial_.read());
        if (value == '!') finishMessage(nowMs);
        else if (receiveLength_ < sizeof(receive_) - 1) receive_[receiveLength_++] = value;
        else receiveLength_ = 0;
    }
    if (nowMs - lastEncoderQueryMs_ >= 50UL) {
        serial_.print(F("$MOTOR_4CH_READ:encoder_20ms!"));
        lastEncoderQueryMs_ = nowMs;
    }
    if (nowMs - lastBatteryQueryMs_ >= 1000UL) {
        serial_.print(F("$MOTOR_4CH_READ:battery!"));
        lastBatteryQueryMs_ = nowMs;
    }
}

DriveCapabilities UartEncoderDriveBackend::capabilities() const { return {true, true, true, true}; }
const DriveFeedback &UartEncoderDriveBackend::feedback() const { return feedback_; }

} // namespace robot

