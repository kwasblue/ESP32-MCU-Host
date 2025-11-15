// modules/IdentityModule.h
#pragma once

#include <Arduino.h>
#include <vector>
#include <cstring>

#include "core/IModule.h"
#include "core/EventBus.h"
#include "core/MultiTransport.h"
#include "core/Protocol.h"
#include "core/Messages.h"
#include "core/Event.h"

class IdentityModule : public IModule {
public:
    IdentityModule(EventBus& bus, MultiTransport& tx, const char* name)
        : bus_(bus), tx_(tx), name_(name) {}

    void setup() override {
        // Send HELLO once at boot
        sendHello();

        // Listen to all events, but only react to WHOAMI_REQUEST
        bus_.subscribe([this](const Event& evt) {
            if (evt.type == EventType::WHOMAI_REQUEST) {
                sendHello();
            }
        });
    }

private:
    void sendHello() {
        Serial.println("[Identity] Sending HELLO");

        std::vector<uint8_t> payload;

        auto push_u8  = [&](uint8_t v) { payload.push_back(v); };
        auto push_u32 = [&](uint32_t v) {
            payload.push_back(static_cast<uint8_t>(v & 0xFF));
            payload.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
            payload.push_back(static_cast<uint8_t>((v >> 16) & 0xFF));
            payload.push_back(static_cast<uint8_t>((v >> 24) & 0xFF));
        };

        const uint8_t  protocol_version = 1;
        const uint8_t  fw_major = 0;
        const uint8_t  fw_minor = 1;
        const uint8_t  fw_patch = 0;
        const uint32_t robot_id = 1;

        push_u8(protocol_version);
        push_u8(fw_major);
        push_u8(fw_minor);
        push_u8(fw_patch);
        push_u32(robot_id);

        uint8_t name_len = static_cast<uint8_t>(strlen(name_));
        push_u8(name_len);
        payload.insert(payload.end(), name_, name_ + name_len);

        uint32_t caps = 0;
        caps |= (1 << 0); // UART
        caps |= (1 << 1); // WIFI
        caps |= (1 << 2); // SPP
        caps |= (1 << 3); // OTA
        push_u32(caps);

        // Build and send frame via MultiTransport
        std::vector<uint8_t> frame;
        Protocol::encode(
            static_cast<uint8_t>(MsgType::HELLO),
            payload.data(),
            payload.size(),
            frame
        );
        tx_.sendBytes(frame.data(), frame.size());
    }

    EventBus&       bus_;
    MultiTransport& tx_;
    const char*     name_;
};
