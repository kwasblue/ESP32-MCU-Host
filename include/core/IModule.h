#pragma once
#include <cstdint>
#include "Event.h"

class IModule {
public:
    virtual ~IModule() = default;

    virtual void setup() {}
    virtual void loop(uint32_t now_ms) {}
    virtual void handleEvent(const Event& evt) {
        (void)evt;
    }
};
