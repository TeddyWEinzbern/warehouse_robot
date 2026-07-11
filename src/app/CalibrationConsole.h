#pragma once

#include <Arduino.h>

#include "subsystems/ArmSubsystem.h"

namespace robot {

class CalibrationConsole {
  public:
    CalibrationConsole();
    void poll(Stream &stream, ArmSubsystem &arm);

  private:
    char line_[20];
    uint8_t length_;
    void execute(ArmSubsystem &arm);
};

} // namespace robot

