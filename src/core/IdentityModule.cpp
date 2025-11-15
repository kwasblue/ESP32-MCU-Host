#include "modules/IdentityModule.h"
#include <Arduino.h>

IdentityModule* IdentityModule::s_instance = nullptr;

void IdentityModule::setup() {
    s_instance = this;
    bus_.subscribe(&IdentityModule::onEventStatic);
}

void IdentityModule::loop(uint32_t /*now_ms*/) {
    // Nothing periodic for now
}

void IdentityModule::onEventStatic(const Event& evt) {
    if (s_instance) {
        s_instance->handleEvent(evt);
    }
}

void IdentityModule::handleEvent(const Event& evt) {
    // This should mirror the logic you previously had in the lambda.
    if (evt.type == EventType::WHOMAI_REQUEST) {
        // Example: respond via transports_ with HELLO / identity info
        Serial.println("[IDENTITY] WHOAMI request received");
        // send HELLO frame, etc.
    }
}
