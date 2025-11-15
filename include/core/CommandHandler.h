#pragma once

#include <string>
#include <ArduinoJson.h>

#include "EventBus.h"
#include "Messages.h"

// Forward declarations if these are in other headers
class ModeManager;
class MotionController;
class SafetyManager;

class CommandHandler {
public:
    CommandHandler(EventBus& bus,
                   ModeManager& mode,
                   MotionController& motion,
                   SafetyManager& safety)
        : bus_(bus), mode_(mode), motion_(motion), safety_(safety) {}

    // Call this once in setup() to attach to the EventBus
    void setup() {
        bus_.subscribe([this](const Event& evt) {
            if (evt.type == EventType::JSON_MESSAGE_RX) {
                this->onJsonCommand(evt.payload.json);
            }
        });
    }

private:
    EventBus&        bus_;
    ModeManager&     mode_;
    MotionController& motion_;
    SafetyManager&   safety_;

    void onJsonCommand(const std::string& jsonStr);

    void handleSetMode(ArduinoJson::JsonVariantConst payload);
    void handleSetVel(ArduinoJson::JsonVariantConst payload);
    void handleStop();
    void handleEstop();
    void handleClearEstop();
    void handleLedOn();
    void handleLedOff();

};
