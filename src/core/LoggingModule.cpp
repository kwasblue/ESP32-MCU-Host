#include "modules/LoggingModule.h"
#include <Arduino.h>

LoggingModule* LoggingModule::s_instance = nullptr;

void LoggingModule::setup() {
    s_instance = this;
    bus_.subscribe(&LoggingModule::onEventStatic);
}

void LoggingModule::loop(uint32_t /*now_ms*/) {
    // If you have periodic log flushing, put it here. For now, do nothing.
}

void LoggingModule::onEventStatic(const Event& evt) {
    if (s_instance) {
        s_instance->handleEvent(evt);
    }
}

void LoggingModule::handleEvent(const Event& evt) {
    // Whatever you previously did in the lambda lives here now.
    // Example (you probably already had something similar):
    if (evt.type == EventType::PING) {
        Serial.println("[LOG] PING event");
    }
    // Extend for other event types as needed.
}
