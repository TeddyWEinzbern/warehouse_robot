#pragma once

#include <Arduino.h>

#include "drivers/DriveBackend.h"

namespace robot {

class UartEncoderDriveBackend : public DriveBackend {
  public:
    explicit UartEncoderDriveBackend(HardwareSerial &serial);
    void begin();
    void setWheelTargets(const WheelTargets &targets);
    void stop();
    void poll(uint32_t nowMs);
    DriveCapabilities capabilities() const;
    const DriveFeedback &feedback() const;

  private:
    HardwareSerial &serial_;
    DriveFeedback feedback_;
    char receive_[64];
    uint8_t receiveLength_;
    uint32_t lastEncoderQueryMs_;
    uint32_t lastBatteryQueryMs_;
    void sendSpeeds(const WheelTargets &targets);
    void finishMessage(uint32_t nowMs);
    bool parseFourLongs(const char *cursor, int32_t *values);
};

} // namespace robot

