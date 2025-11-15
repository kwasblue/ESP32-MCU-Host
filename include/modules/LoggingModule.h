#pragma once
#include <Arduino.h>
#include "core/IModule.h"
#include "core/EventBus.h"

class LoggingModule : public IModule {
public:
    explicit LoggingModule(EventBus& bus)
        : bus_(bus) {}

    void setup() override {
        bus_.subscribe([this](const Event& evt) { this->handleEvent(evt); });
    }

    void handleEvent(const Event& evt) override {
        switch (evt.type) {
        case EventType::HEARTBEAT:
            Serial.print("[MCU] HEARTBEAT @ ");
            Serial.print(evt.timestamp_ms);
            Serial.println(" ms");
            break;
        case EventType::PING:
            Serial.print("[MCU] Received PING @ ");
            Serial.println(evt.timestamp_ms);
            break;
        case EventType::PONG:
            Serial.print("[MCU] Received PONG @ ");
            Serial.println(evt.timestamp_ms);
            break;
        default:
            break;
        }
    }

private:
    EventBus& bus_;
};
