#include "core/MCUHost.h"
#include "core/MessageRouter.h"
#include <Arduino.h>

// Define the static instance pointer
MCUHost* MCUHost::s_instance = nullptr;

MCUHost::MCUHost(EventBus& bus, MessageRouter* router)
    : bus_(bus)
    , router_(router)
{
    s_instance = this;
}

void MCUHost::addModule(IModule* module) {
    if (module) {
        modules_.push_back(module);
    }
}

void MCUHost::setup() {
    // Let each module set itself up (and subscribe to the bus)
    for (auto* m : modules_) {
        if (m) m->setup();
    }

    // Subscribe host to the EventBus via static trampoline
    bus_.subscribe(&MCUHost::onEventStatic);
}

void MCUHost::loop(uint32_t now_ms) {
    // Let router pump incoming frames
    if (routerLoop_) {
        routerLoop_();
    }

    // Let modules run
    for (auto* m : modules_) {
        if (m) m->loop(now_ms);
    }

    // Optional safety/wd logic
    runSafety(now_ms);
}

void MCUHost::onEventStatic(const Event& evt) {
    if (s_instance) {
        s_instance->onEvent(evt);
    }
}

void MCUHost::onEvent(const Event& evt) {
    if (evt.type == EventType::HEARTBEAT) {
        lastHeartbeatMs_ = evt.timestamp_ms;
        // Later: you can add watchdog or diagnostics here
    }
}

void MCUHost::runSafety(uint32_t now_ms) {
    (void)now_ms;
    // Placeholder for future safety checks
}
