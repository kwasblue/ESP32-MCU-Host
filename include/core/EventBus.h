#pragma once
#include <cstdint>
#include "core/Event.h"

class EventBus {
public:
    using Handler = void (*)(const Event&);

    void subscribe(Handler h) {
        if (handlerCount_ < MAX_HANDLERS && h != nullptr) {
            handlers_[handlerCount_++] = h;
        }
        // else: silently ignore extra handlers, or add a debug print if you want
    }

    void publish(const Event& evt) {
        for (uint8_t i = 0; i < handlerCount_; ++i) {
            Handler h = handlers_[i];
            if (h) {
                h(evt);
            }
        }
    }

private:
    static constexpr uint8_t MAX_HANDLERS = 8;  // adjust if you have more modules

    Handler  handlers_[MAX_HANDLERS]{};
    uint8_t  handlerCount_ = 0;
};
