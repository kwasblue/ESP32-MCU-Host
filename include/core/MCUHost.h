#pragma once
#include <vector>
#include <functional>

#include "core/EventBus.h"
#include "core/Event.h"
#include "core/IModule.h"

class MessageRouter;  // forward-declare

class MCUHost {
public:
    MCUHost(EventBus& bus, MessageRouter* router = nullptr);

    void addModule(IModule* module);

    void setup();
    void loop(uint32_t now_ms);

    EventBus& bus() { return bus_; }

    void setRouterLoop(std::function<void()> fn) {
        routerLoop_ = std::move(fn);
    }

private:
    EventBus&             bus_;
    MessageRouter*        router_ = nullptr;
    std::vector<IModule*> modules_;
    uint32_t              lastHeartbeatMs_ = 0;
    std::function<void()> routerLoop_;

    void runSafety(uint32_t now_ms);

    // === Static trampoline for EventBus ===
    static MCUHost* s_instance;
    static void onEventStatic(const Event& evt);
    void onEvent(const Event& evt);
};
