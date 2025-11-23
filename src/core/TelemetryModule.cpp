#include "modules/TelemetryModule.h"
#include "core/Debug.h"

TelemetryModule::TelemetryModule(EventBus& bus)
    : bus_(bus) {}

// Nothing to subscribe to for now; this module only publishes periodically
void TelemetryModule::setup() {
    DBG_PRINTLN("[TelemetryModule] setup complete");
}

void TelemetryModule::loop(uint32_t now_ms) {
    // 0 = disabled
    if (intervalMs_ == 0) {
        return;
    }

    if (now_ms - lastTickMs_ < intervalMs_) {
        return;
    }

    lastTickMs_ = now_ms;
    sendTelemetry(now_ms);
}


void TelemetryModule::setInterval(uint32_t intervalMs) {
    intervalMs_ = intervalMs;
    lastTickMs_ = 0;  // or lastTickMs_ = millis() if you pass it in
}

void TelemetryModule::registerProvider(const char* name, ProviderFn fn) {
    if (!name || !fn) return;

    Provider p;
    p.name = name;
    p.fn   = std::move(fn);
    providers_.push_back(std::move(p));

    DBG_PRINTF("[TelemetryModule] registered provider '%s'\n", name);
}

void TelemetryModule::sendTelemetry(uint32_t now_ms) {
    using namespace ArduinoJson;

    if (providers_.empty()) {
        // Nothing to send
        return;
    }

    JsonDocument doc;   // dynamic doc (ArduinoJson v7)

    doc["src"]   = "mcu";
    doc["type"]  = "TELEMETRY";
    doc["ts_ms"] = now_ms;

    JsonObject data = doc["data"].to<JsonObject>();

    // Ask each provider to fill its own subtree: data[name] = {...}
    for (auto& p : providers_) {
        JsonObject node = data[p.name.c_str()].to<JsonObject>();
        // Provider fills its own object
        p.fn(node);
    }

    std::string out;
    serializeJson(doc, out);

    // Wrap as JSON_MESSAGE_TX so MessageRouter sends it to Python
    Event evt;
    evt.type         = EventType::JSON_MESSAGE_TX;
    evt.timestamp_ms = now_ms;
    evt.payload      = {};
    evt.payload.json = std::move(out);

    bus_.publish(evt);
}
