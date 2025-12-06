// managers/LimitSwitchManager.h
#pragma once
#include <vector>
#include "core/EventBus.h"

struct LimitSwitchConfig {
    uint8_t  id;
    uint8_t  pin;
    bool     activeHigh;     // true if HIGH = triggered
    uint32_t debounceMs;
};

class LimitSwitchManager {
public:
    LimitSwitchManager(EventBus& bus) : bus_(bus) {}

    void addSwitch(const LimitSwitchConfig& cfg) {
        switches_.push_back({cfg, false, 0});
        pinMode(cfg.pin, cfg.activeHigh ? INPUT_PULLDOWN : INPUT_PULLUP);
    }

    void poll() {
        const uint32_t now = millis();
        for (auto& s : switches_) {
            int raw = digitalRead(s.cfg.pin);
            bool active = s.cfg.activeHigh ? (raw == HIGH) : (raw == LOW);

            if (active != s.lastState && (now - s.lastChangeMs) > s.cfg.debounceMs) {
                s.lastState = active;
                s.lastChangeMs = now;
                if (active) {
                    publishTriggered(s.cfg.id);
                } else {
                    publishCleared(s.cfg.id);
                }
            }
        }
    }

private:
    struct SwitchState {
        LimitSwitchConfig cfg;
        bool              lastState;
        uint32_t          lastChangeMs;
    };

    EventBus& bus_;
    std::vector<SwitchState> switches_;

    void publishTriggered(uint8_t id);
    void publishCleared(uint8_t id);
};
