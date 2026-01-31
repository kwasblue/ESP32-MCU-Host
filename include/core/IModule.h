#pragma once
#include <cstdint>
#include "core/Event.h"

class IModule {
public:
    virtual ~IModule() = default;

    virtual void setup() {}
    virtual void loop(uint32_t now_ms) {}
    virtual const char* name() const = 0;
    virtual void handleEvent(const Event& evt) {
        (void)evt;
    }
};
