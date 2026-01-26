#include "modules/IdentityModule.h"
#include "core/Debug.h"
#include <Arduino.h>
#include <ArduinoJson.h>

#include "config/Version.h"   // auto-generated Version::*

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

static void publishIdentity(EventBus& bus) {
    JsonDocument doc;
    doc["kind"]     = "identity";
    doc["protocol"] = Version::PROTOCOL;
    doc["firmware"] = Version::FIRMWARE;
    doc["board"]    = Version::BOARD;
    doc["name"]     = Version::NAME;

    std::string out;
    serializeJson(doc, out);

    Event tx{};
    tx.type = EventType::JSON_MESSAGE_TX;
    tx.timestamp_ms = 0;          // optional; set millis() if you want
    tx.payload.json = out;
    bus.publish(tx);
}

void IdentityModule::handleEvent(const Event& evt) {
    if (evt.type == EventType::WHOMAI_REQUEST) {
        DBG_PRINTLN("[IDENTITY] WHOAMI request received");
        publishIdentity(bus_);
    }
}
