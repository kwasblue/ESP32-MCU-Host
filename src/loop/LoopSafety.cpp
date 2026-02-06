#include "core/ServiceContext.h"
#include "command/ModeManager.h"

namespace mcu {

void runSafetyLoop(ServiceContext& ctx, uint32_t now_ms) {
    if (ctx.mode) {
        ctx.mode->update(now_ms);
    }
}

} // namespace mcu
