#pragma once

#include "domain/RobotTypes.h"

namespace robot {

inline bool calibrationActionsReady(
    bool platformInitialized, const DriveHealth &drive
) {
    return platformInitialized && drive.initialized;
}

} // namespace robot
