#pragma once

#include "core/IModule.h"
#include "core/EventBus.h"
#include "core/Event.h"
#include "core/MultiTransport.h"

class IdentityModule : public IModule {
public:
    IdentityModule(EventBus& bus,
                   MultiTransport& transports,
                   const char* name)
        : bus_(bus)
        , transports_(transports)
        , name_(name) {}

    void setup() override;
    void loop(uint32_t now_ms) override;

private:
    EventBus&       bus_;
    MultiTransport& transports_;
    const char*     name_;

    static IdentityModule* s_instance;
    static void onEventStatic(const Event& evt);

    // ✅ Fix warning: explicitly mark override
    void handleEvent(const Event& evt) override;
};
