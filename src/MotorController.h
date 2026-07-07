#pragma once

#include <Arduino.h>

#include "Config.h"
#include "State.h"

class MotorController {
  public:
    MotorController();

    void begin();
    void stop();
    void drive(int forward, int turn, int strafe);
    void setWheelSpeeds(
        int frontLeft, int frontRight, int rearLeft, int rearRight
    );
    void setSingleMotor(uint8_t motorIndex, int speed);

    const WheelSpeedCommand &lastWheelSpeeds() const;
    bool isStopped() const;

  private:
    uint8_t latchState_;
    WheelSpeedCommand lastWheelSpeeds_;
    bool stopped_;

    void updateLatch();
    void setLatchBit(uint8_t bit, bool enabled);
    void setMotorSpeed(const Config::MotorConfig &motor, float speed);
    void normalize(
        float &frontLeft, float &frontRight, float &rearLeft, float &rearRight
    ) const;
    void driveTank(int forward, int turn);
    void driveMecanum(int forward, int turn, int strafe);
    uint8_t speedToPwm(const Config::MotorConfig &motor, float speed) const;
};
