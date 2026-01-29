#include "core/MessageRouter.h"
#include "core/Debug.h"
#include "config/Version.h"
#include <ArduinoJson.h>

MessageRouter* MessageRouter::s_instance = nullptr;

MessageRouter::MessageRouter(EventBus& bus, ITransport& transport)
    : bus_(bus)
    , transport_(transport)
{
    s_instance = this;
}

void MessageRouter::setup() {
    transport_.setFrameHandler(
        [this](const uint8_t* frame, size_t len) { this->onFrame(frame, len); }
    );
    transport_.begin();

    bus_.subscribe(&MessageRouter::onEventStatic);

    txBuffer_.reserve(64);
}

void MessageRouter::loop() {
    transport_.loop();
}

void MessageRouter::onEventStatic(const Event& evt) {
    if (s_instance) {
        s_instance->onEvent(evt);
    }
}

void MessageRouter::onFrame(const uint8_t* frame, size_t len) {
    if (len == 0) return;
    uint8_t msgType = frame[0];
    uint32_t now_ms = millis();

    switch (msgType) {
    case Protocol::MSG_PING: {
        Event evt{ EventType::PING, now_ms, EventPayload{} };
        bus_.publish(evt);
        sendSimple(Protocol::MSG_PONG);
        break;
    }
    
    case Protocol::MSG_PONG: {
        Event evt{ EventType::PONG, now_ms, EventPayload{} };
        bus_.publish(evt);
        break;
    }

    case Protocol::MSG_VERSION_REQUEST: {
        DBG_PRINTLN("[Router] VERSION_REQUEST received");
        sendVersionResponse();
        break;
    }

    case Protocol::MSG_WHOAMI: {
        Event evt;
        evt.type         = EventType::WHOMAI_REQUEST;
        evt.timestamp_ms = now_ms;
        evt.payload      = {};
        bus_.publish(evt);
        break;
    }

    case Protocol::MSG_CMD_JSON: {
        if (len <= 1) {
            return;
        }
        const uint8_t* jsonData = frame + 1;
        size_t jsonLen          = len - 1;

        std::string jsonStr(reinterpret_cast<const char*>(jsonData), jsonLen);

        Event evt;
        evt.type         = EventType::JSON_MESSAGE_RX;
        evt.timestamp_ms = now_ms;
        evt.payload      = {};
        evt.payload.json = std::move(jsonStr);

        bus_.publish(evt);
        break;
    }

    default:
        DBG_PRINTF("[Router] Unknown msgType: 0x%02X\n", msgType);
        break;
    }
}

void MessageRouter::onEvent(const Event& evt) {
    switch (evt.type) {
    case EventType::HEARTBEAT:
        sendSimple(Protocol::MSG_HEARTBEAT);
        break;

    case EventType::JSON_MESSAGE_TX: {
        const std::string& json = evt.payload.json;
        if (json.empty()) {
            DBG_PRINTLN("[Router] JSON_MESSAGE_TX with empty payload");
            return;
        }

        DBG_PRINTF("[Router] TX JSON (%u bytes): %s\n",
                   (unsigned)json.size(), json.c_str());

        txBuffer_.clear();
        Protocol::encode(
            Protocol::MSG_CMD_JSON,
            reinterpret_cast<const uint8_t*>(json.data()),
            json.size(),
            txBuffer_
        );

        if (!txBuffer_.empty()) {
            transport_.sendBytes(txBuffer_.data(), txBuffer_.size());
        } else {
            DBG_PRINTLN("[Router] encode() produced empty buffer");
        }
        break;
    }
    case EventType::BIN_MESSAGE_TX: {
        const std::vector<uint8_t>& bin = evt.payload.bin;
        if (bin.empty()) {
            DBG_PRINTLN("[Router] BIN_MESSAGE_TX with empty payload");
            return;
        }

        DBG_PRINTF("[Router] TX BIN (%u bytes)\n", (unsigned)bin.size());

        txBuffer_.clear();
        Protocol::encode(
            Protocol::MSG_TELEMETRY_BIN,
            bin.data(),
            bin.size(),
            txBuffer_
        );

        if (!txBuffer_.empty()) {
            transport_.sendBytes(txBuffer_.data(), txBuffer_.size());
        } else {
            DBG_PRINTLN("[Router] encode() produced empty buffer");
        }
        break;
    }
    default:
        break;
    }
}

void MessageRouter::sendSimple(uint8_t msgType) {
    txBuffer_.clear();                 // ✅ add this
    Protocol::encode(msgType, nullptr, 0, txBuffer_);
    if (!txBuffer_.empty()) {
        transport_.sendBytes(txBuffer_.data(), txBuffer_.size());
    }
}

void MessageRouter::sendVersionResponse() {
    JsonDocument doc;
    doc["firmware"] = Version::FIRMWARE;
    doc["protocol"] = Version::PROTOCOL;
    doc["board"]    = Version::BOARD;
    doc["name"]     = Version::NAME;

    std::string json;
    serializeJson(doc, json);

    DBG_PRINTF("[Router] VERSION_RESPONSE: %s\n", json.c_str());

    txBuffer_.clear();
    Protocol::encode(
        Protocol::MSG_VERSION_RESPONSE,
        reinterpret_cast<const uint8_t*>(json.data()),
        json.size(),
        txBuffer_
    );

    if (!txBuffer_.empty()) {
        transport_.sendBytes(txBuffer_.data(), txBuffer_.size());
    }
}