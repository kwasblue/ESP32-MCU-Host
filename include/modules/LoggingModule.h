#pragma once
#include "core/IModule.h"
#include "core/EventBus.h"
#include "core/Event.h"

class LoggingModule : public IModule {
public:
    explicit LoggingModule(EventBus& bus)
        : bus_(bus) {}

    void setup() override;
    void loop(uint32_t now_ms) override;

private:
    EventBus& bus_;

    static LoggingModule* s_instance;
    static void onEventStatic(const Event& evt);
    void handleEvent(const Event& evt);
};
