// include/command/CommandContext.h
// Shared context passed to command handlers

#pragma once

#include <string>
#include <ArduinoJson.h>
#include "config/CommandDefs.h"
#include "core/EventBus.h"
#include "core/Event.h"
#include "command/ModeManager.h"

/**
 * Context passed to command handlers.
 * Provides access to shared state and helper methods for ACK/error responses.
 */
struct CommandContext {
    // Current command metadata
    uint32_t seq = 0;
    CmdType cmdType = CmdType::UNKNOWN;
    bool wantAck = true;

    // Core dependencies
    EventBus& bus;
    ModeManager& mode;

    // ACK cache for duplicate detection
    static constexpr int kAckCacheSize = 8;
    struct AckCacheEntry {
        bool valid = false;
        uint32_t seq = 0;
        CmdType cmdType = CmdType::UNKNOWN;
        std::string ackJson;
    };
    AckCacheEntry ackCache[kAckCacheSize];
    int ackCacheWrite = 0;

    CommandContext(EventBus& b, ModeManager& m) : bus(b), mode(m) {}

    // -------------------------------------------------------------------------
    // Response Helpers
    // -------------------------------------------------------------------------

    void sendAck(const char* cmd, bool ok, JsonDocument& resp) {
        if (!wantAck) return;

        resp["src"] = "mcu";
        resp["cmd"] = cmd;
        resp["ok"] = ok;
        resp["seq"] = seq;

        std::string out;
        serializeJson(resp, out);

        storeAck(cmdType, seq, out);
        publishJson(std::move(out));
    }

    void sendError(const char* cmd, const char* error) {
        if (!wantAck) return;

        JsonDocument resp;
        resp["src"] = "mcu";
        resp["cmd"] = cmd;
        resp["ok"] = false;
        resp["error"] = error;
        resp["seq"] = seq;

        std::string out;
        serializeJson(resp, out);
        storeAck(cmdType, seq, out);
        publishJson(std::move(out));
    }

    // -------------------------------------------------------------------------
    // State Guards
    // -------------------------------------------------------------------------

    bool requireIdle(const char* cmdName) {
        if (mode.mode() == RobotMode::IDLE) {
            return true;
        }
        sendError(cmdName, "not_idle");
        return false;
    }

    bool requireArmed(const char* cmdName) {
        if (mode.canMove()) {
            return true;
        }
        sendError(cmdName, "not_armed");
        return false;
    }

    // -------------------------------------------------------------------------
    // ACK Cache (for duplicate command detection)
    // -------------------------------------------------------------------------

    bool tryReplayAck(CmdType cmd, uint32_t cmdSeq) {
        if (!wantAck) return false;
        if (cmdSeq == 0) return false;

        for (int i = 0; i < kAckCacheSize; ++i) {
            auto& e = ackCache[i];
            if (e.valid && e.seq == cmdSeq && e.cmdType == cmd) {
                publishJsonCopy(e.ackJson);
                return true;
            }
        }
        return false;
    }

private:
    void storeAck(CmdType cmd, uint32_t cmdSeq, const std::string& ackJson) {
        if (cmdSeq == 0) return;

        auto& e = ackCache[ackCacheWrite];
        e.valid = true;
        e.seq = cmdSeq;
        e.cmdType = cmd;
        e.ackJson = ackJson;
        ackCacheWrite = (ackCacheWrite + 1) % kAckCacheSize;
    }

    void publishJson(std::string&& out) {
        Event evt;
        evt.type = EventType::JSON_MESSAGE_TX;
        evt.payload.json = std::move(out);
        bus.publish(evt);
    }

    void publishJsonCopy(const std::string& out) {
        std::string copy = out;
        publishJson(std::move(copy));
    }
};
