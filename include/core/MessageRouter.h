#pragma once
#include <vector>
#include <Arduino.h>
#include "EventBus.h"
#include "ITransport.h"
#include "Protocol.h"

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
