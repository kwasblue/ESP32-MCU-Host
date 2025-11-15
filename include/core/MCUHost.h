#pragma once
#include <vector>
#include <functional>
#include "EventBus.h"
#include "IModule.h"

class MessageRouter;  // forward-declare

class MCUHost {
public:
    MCUHost(EventBus& bus, MessageRouter* router = nullptr)
        : bus_(bus), router_(router) {}

    void addModule(IModule* module) {
        modules_.push_back(module);
    }

    void setup() {
        for (auto* m : modules_) {
            if (m) m->setup();
        }

        bus_.subscribe([this](const Event& evt) {
            this->handleEvent(evt);
        });
    }

    void loop(uint32_t now_ms) {
        if (routerLoop_) {
            routerLoop_();
        }

        for (auto* m : modules_) {
            if (m) m->loop(now_ms);
        }

        runSafety(now_ms);
    }

    EventBus& bus() { return bus_; }

    void setRouterLoop(std::function<void()> fn) {
        routerLoop_ = std::move(fn);
    }

private:
    EventBus&                 bus_;
    MessageRouter*            router_ = nullptr;
    std::vector<IModule*>     modules_;
    uint32_t                  lastHeartbeatMs_ = 0;
    std::function<void()>     routerLoop_;

    void handleEvent(const Event& evt) {
        if (evt.type == EventType::HEARTBEAT) {
            lastHeartbeatMs_ = evt.timestamp_ms;
        }
    }

    void runSafety(uint32_t now_ms) {
        (void)now_ms;
        // later: safety checks here
    }
};
