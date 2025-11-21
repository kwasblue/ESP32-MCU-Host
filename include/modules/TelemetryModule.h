#pragma once

#include "modules/IdentityModule.h"
#include "core/EventBus.h"
#include "core/Event.h"

#include <ArduinoJson.h>
#include <functional>
#include <string>
#include <vector>

// TelemetryModule:
// - Runs as an IModule inside MCUHost
// - Aggregates telemetry from multiple "providers" into one JSON packet
// - Emits that packet as JSON_MESSAGE_TX so MessageRouter sends it to Python
class TelemetryModule : public IModule {
public:
    // Callback receives a JsonObject to fill with its own data:
    // e.g. node["vx"] = ..., node["omega"] = ...
    using ProviderFn = std::function<void(ArduinoJson::JsonObject&)>;

    explicit TelemetryModule(EventBus& bus);

    // IModule interface
    void setup() override;
    void loop(uint32_t now_ms) override;

    // Register a telemetry provider under a name, e.g. "motion", "system", "imu"
    // Example:
    //   telemetry.registerProvider("system", [&](JsonObject& node) {
    //       node["uptime_ms"] = millis();
    //   });
    void registerProvider(const char* name, ProviderFn fn);

    // Set telemetry interval in ms (default: 100 ms == 10 Hz)
    void setInterval(uint32_t intervalMs);

private:
    struct Provider {
        std::string name;
        ProviderFn  fn;
    };

    EventBus&            bus_;
    std::vector<Provider> providers_;

    uint32_t lastTickMs_  = 0;
    uint32_t intervalMs_  = 100;  // default 10 Hz

    void sendTelemetry(uint32_t now_ms);
};
