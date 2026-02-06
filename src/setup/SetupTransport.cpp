#include <Arduino.h>

#include "setup/ISetupModule.h"
#include "core/ServiceContext.h"
#include "core/IModule.h"
#include "command/MessageRouter.h"
#include "command/CommandRegistry.h"
#include "core/MCUHost.h"

namespace {

class SetupTransportModule : public mcu::ISetupModule {
public:
    const char* name() const override { return "Transport"; }
    bool isCritical() const override { return true; }  // No transport = no commands = unsafe

    mcu::Result<void> setup(mcu::ServiceContext& ctx) override {
        if (!ctx.commands || !ctx.router || !ctx.host) {
            return mcu::Result<void>::err(mcu::ErrorCode::NotInitialized);
        }

        // Note: Handlers are already registered in ServiceStorage.initCommands()

        ctx.commands->setup();
        ctx.router->setup();

        // Set up router loop callback
        ctx.host->setRouterLoop([ctx]() {
            if (ctx.router) {
                ctx.router->loop();
            }
        });

        // Add modules to host
        if (ctx.heartbeat) ctx.host->addModule(ctx.heartbeat);
        if (ctx.logger)    ctx.host->addModule(ctx.logger);
        if (ctx.identity)  ctx.host->addModule(ctx.identity);

        ctx.host->setup();

        Serial.println("[TRANSPORT] Router and host configured");

        return mcu::Result<void>::ok();
    }
};

SetupTransportModule g_setupTransport;

} // anonymous namespace

mcu::ISetupModule* getSetupTransportModule() {
    return &g_setupTransport;
}
