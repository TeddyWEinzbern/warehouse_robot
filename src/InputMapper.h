#pragma once

#include "State.h"

class InputMapper {
    public:
    RobotCommand map(const RobotCommand &command) const;
};
