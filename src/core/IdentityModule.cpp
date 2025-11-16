#include "modules/IdentityModule.h"
#include "core/Debug.h"      // <-- add this
#include <Arduino.h>

IdentityModule* IdentityModule::s_instance = nullptr;

void IdentityModule::setup() {
    s_instance = this;
    bus_.subscribe(&IdentityModule::onEventStatic);

    DBG_PRINTLN("[IdentityModule] setup complete");
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
    if (evt.type == EventType::WHOMAI_REQUEST) {
        DBG_PRINTLN("[IDENTITY] WHOAMI request received");

        // TODO: Send identity/HELLO frame if desired.
        // Example (uncomment when implemented):
        // transport_.sendHello("ESP32-BOT", FW_VERSION);
    }
}

