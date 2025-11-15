#include "core/MessageRouter.h"

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

    // Subscribe via static trampoline
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

    case static_cast<uint8_t>(MsgType::WHOAMI): {
        Event evt;
        evt.type         = EventType::WHOMAI_REQUEST;
        evt.timestamp_ms = now_ms;
        evt.payload      = {};
        bus_.publish(evt);
        break;
    }

    case static_cast<uint8_t>(MsgType::CMD_JSON): {
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
        break;
    }
}

void MessageRouter::onEvent(const Event& evt) {
    switch (evt.type) {
    case EventType::HEARTBEAT:
        sendSimple(Protocol::MSG_HEARTBEAT);
        break;
    default:
        break;
    }
}

void MessageRouter::sendSimple(uint8_t msgType) {
    Protocol::encode(msgType, nullptr, 0, txBuffer_);
    if (!txBuffer_.empty()) {
        transport_.sendBytes(txBuffer_.data(), txBuffer_.size());
    }
}
