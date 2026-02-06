#include "core/LoopTiming.h"

namespace mcu {

static LoopTiming g_loopTiming;

LoopTiming& getLoopTiming() {
    return g_loopTiming;
}

} // namespace mcu
