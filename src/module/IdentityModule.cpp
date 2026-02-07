#include "module/IdentityModule.h"
#include "core/Debug.h"
#include <Arduino.h>
#include <ArduinoJson.h>

#include "config/Version.h"         // auto-generated Version::*
#include "config/DeviceManifest.h"  // unified capabilities

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
    using namespace mcu;

    // Build unified capability mask
    uint32_t caps = buildDeviceCaps();

    JsonDocument doc;
    doc["kind"]           = "identity";
    doc["protocol"]       = Version::PROTOCOL;
    doc["schema_version"] = Version::SCHEMA_VERSION;
    doc["firmware"]       = Version::FIRMWARE;
    doc["board"]          = Version::BOARD;
    doc["name"]           = Version::NAME;
    doc["capabilities"]   = caps;

    // Feature array for human-readable capability list
    JsonArray features = doc["features"].to<JsonArray>();

    // Iterate through all capability bits and add enabled ones
    static constexpr uint32_t CAP_BITS[] = {
        DeviceCap::BINARY_PROTOCOL, DeviceCap::INTENT_BUFFERING,
        DeviceCap::STATE_SPACE_CTRL, DeviceCap::OBSERVERS,
        DeviceCap::UART, DeviceCap::WIFI, DeviceCap::BLE, DeviceCap::MQTT,
        DeviceCap::DC_MOTOR, DeviceCap::SERVO, DeviceCap::STEPPER, DeviceCap::MOTION_CTRL,
        DeviceCap::ENCODER, DeviceCap::IMU, DeviceCap::LIDAR, DeviceCap::ULTRASONIC,
        DeviceCap::SIGNAL_BUS, DeviceCap::CONTROL_KERNEL, DeviceCap::OBSERVER,
        DeviceCap::TELEMETRY, DeviceCap::SAFETY, DeviceCap::AUDIO
    };

    for (uint32_t bit : CAP_BITS) {
        if (caps & bit) {
            features.add(capBitToName(bit));
        }
    }

    // Add loop rates
    const auto& rates = getLoopRates();
    JsonObject timing = doc["timing"].to<JsonObject>();
    timing["control_hz"] = rates.ctrl_hz;
    timing["telemetry_hz"] = rates.telem_hz;
    timing["safety_hz"] = rates.safety_hz;

    std::string out;
    serializeJson(doc, out);

    Event tx{};
    tx.type = EventType::JSON_MESSAGE_TX;
    tx.timestamp_ms = 0;
    tx.payload.json = out;
    bus.publish(tx);
}

void IdentityModule::handleEvent(const Event& evt) {
    if (evt.type == EventType::WHOMAI_REQUEST) {
        DBG_PRINTLN("[IDENTITY] WHOAMI request received");
        publishIdentity(bus_);
    }
}
