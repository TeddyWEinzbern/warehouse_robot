#pragma once

#include <stdint.h>

namespace robot {

// Qualification-only stack watermark. The reported value excludes a
// deliberately unpainted 48-byte gap below the stack pointer, so it is a
// conservative lower bound on the minimum free stack/heap separation.
void beginStackWatermark();
uint16_t minimumFreeStackBytes();

} // namespace robot
