#pragma once
#include <vector>
#include <Arduino.h>
#include "EventBus.h"
#include "ITransport.h"
#include "Protocol.h"
#include "Messages.h"

class MessageRouter {
public:
    MessageRouter(EventBus& bus, ITransport& transport)
        : bus_(bus), transport_(transport) {}

    void setup() {
        transport_.setFrameHandler(
            [this](const uint8_t* frame, size_t len) { this->onFrame(frame, len); }
        );
        transport_.begin();

        bus_.subscribe([this](const Event& evt) { this->onEvent(evt); });

        txBuffer_.reserve(64);
    }

    void loop() {
        transport_.loop();
    }

private:
    EventBus&            bus_;
    ITransport&          transport_;
    std::vector<uint8_t> txBuffer_;

    void onFrame(const uint8_t* frame, size_t len) {
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
        // In MessageRouter.cpp (inside switch(msg_type))
        case static_cast<uint8_t>(MsgType::WHOAMI): {
            Event evt;
            evt.type         = EventType::WHOMAI_REQUEST;
            evt.timestamp_ms = now_ms;
            evt.payload      = {};  // nothing needed for now
            bus_.publish(evt);
            break;
        }

        // === NEW: high-level JSON command case ===
        case static_cast<uint8_t>(MsgType::CMD_JSON): {
            if (len <= 1) {
                // no payload, ignore or log
                return;
            }

            // frame[0] = msgType, rest is JSON payload
            const uint8_t* jsonData = frame + 1;
            size_t jsonLen          = len - 1;

            std::string jsonStr(reinterpret_cast<const char*>(jsonData), jsonLen);

            Event evt;
            evt.type         = EventType::JSON_MESSAGE_RX;
            evt.timestamp_ms = now_ms;
            evt.payload      = {};
            evt.payload.json = std::move(jsonStr);   // store raw JSON in payload

            bus_.publish(evt);
            break;
        }

        default:
            break;
        }
    }

    void onEvent(const Event& evt) {
        switch (evt.type) {
        case EventType::HEARTBEAT:
            sendSimple(Protocol::MSG_HEARTBEAT);
            break;
        default:
            break;
        }
    }

    void sendSimple(uint8_t msgType) {
        Protocol::encode(msgType, nullptr, 0, txBuffer_);
        if (!txBuffer_.empty()) {
            transport_.sendBytes(txBuffer_.data(), txBuffer_.size());
        }
    }
};
