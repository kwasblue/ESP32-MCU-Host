#include "module/IdentityModule.h"
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
    doc["kind"]           = "identity";
    doc["protocol"]       = Version::PROTOCOL;
    doc["schema_version"] = Version::SCHEMA_VERSION;
    doc["firmware"]       = Version::FIRMWARE;
    doc["board"]          = Version::BOARD;
    doc["name"]           = Version::NAME;
    doc["capabilities"]   = Version::CAPABILITIES;

    // Feature array for human-readable capability list
    JsonArray features = doc["features"].to<JsonArray>();
    if (Version::CAPABILITIES & Version::Caps::BINARY_PROTOCOL) {
        features.add("binary_protocol");
    }
    if (Version::CAPABILITIES & Version::Caps::INTENT_BUFFERING) {
        features.add("intent_buffering");
    }
    if (Version::CAPABILITIES & Version::Caps::STATE_SPACE_CTRL) {
        features.add("state_space_ctrl");
    }
    if (Version::CAPABILITIES & Version::Caps::OBSERVERS) {
        features.add("observers");
    }

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
