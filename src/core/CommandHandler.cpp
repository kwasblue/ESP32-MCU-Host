#include "core/CommandHandler.h"
#include "core/Messages.h"
#include "core/ModeManager.h"
#include "core/MotionController.h"
#include "core/SafetyManager.h"
#include "config/PinConfig.h"
#include <Arduino.h>   // for strcmp, millis, etc.

using namespace ArduinoJson;

void CommandHandler::onJsonCommand(const std::string& jsonStr) {
    ArduinoJson::JsonDocument doc;
    auto err = deserializeJson(doc, jsonStr);
    if (err) {
        // TODO: publish a LOG or ERROR event if you want
        // e.g., LOG: "JSON parse failed: <err.c_str()>"
        return;
    }

    const char* kindStr = doc["kind"] | "cmd";
    const char* typeStr = doc["type"] | "UNKNOWN";

    MsgKind kind   = msgKindFromString(kindStr);
    CmdType cmd    = cmdTypeFromString(typeStr);

    if (kind != MsgKind::CMD) {
        // For now, ignore non-command JSON messages
        return;
    }

    JsonVariantConst payload = doc["payload"];

    switch (cmd) {
        case CmdType::SET_MODE:
            handleSetMode(payload);
            break;
        case CmdType::SET_VEL:
            handleSetVel(payload);
            break;
        case CmdType::STOP:
            handleStop();
            break;
        case CmdType::ESTOP:
            handleEstop();
            break;
        case CmdType::CLEAR_ESTOP:
            handleClearEstop();
            break;
        case CmdType::LED_ON:
            handleLedOn();
            break;

        case CmdType::LED_OFF:
            handleLedOff();
            break;

        default:
            // Unknown command type – ignore or log
            break;
    }
}

void CommandHandler::handleSetMode(JsonVariantConst payload) {
    const char* modeStr = payload["mode"] | "IDLE";

    // Ask ModeManager what modes are
    RobotMode newMode = mode_.mode();

    if (strcmp(modeStr, "IDLE") == 0) {
        newMode = RobotMode::IDLE;
    } else if (strcmp(modeStr, "ARMED") == 0) {
        newMode = RobotMode::ARMED;
    } else if (strcmp(modeStr, "ACTIVE") == 0) {
        newMode = RobotMode::ACTIVE;
    } else {
        // unsupported mode string
        return;
    }

    // If we're in ESTOP, only CLEAR_ESTOP is allowed to change state
    if (mode_.mode() == RobotMode::ESTOP) {
        return;
    }

    mode_.setMode(newMode);
    // TODO: optionally publish a STATUS event with new mode
}

void CommandHandler::handleSetVel(JsonVariantConst payload) {
    // Basic safety gates
    if (!mode_.canMove() || safety_.isEstopActive()) {
        return;
    }

    float vx    = payload["vx"]    | 0.0f;
    float omega = payload["omega"] | 0.0f;

    // Optionally clamp values here, e.g.:
    // vx    = constrain(vx, -0.5f, 0.5f);
    // omega = constrain(omega, -1.0f, 1.0f);

    motion_.setVelocity(vx, omega);

    // TODO later:
    // - store last command time for timeout safety (zero velocity if stale)
}

void CommandHandler::handleStop() {
    motion_.stop();
}

void CommandHandler::handleEstop() {
    safety_.estop();
    motion_.stop();
    mode_.setMode(RobotMode::ESTOP);
    // TODO: publish STATUS_ESTOP if you want
}

void CommandHandler::handleClearEstop() {
    safety_.clearEstop();
    motion_.stop();
    mode_.setMode(RobotMode::IDLE);
}

void CommandHandler::handleLedOn() {
    Serial.println("[CMD] LED ON");
    digitalWrite(Pins::LED_STATUS, HIGH);
}

void CommandHandler::handleLedOff() {
    Serial.println("[CMD] LED OFF");
    digitalWrite(Pins::LED_STATUS, LOW);
}
