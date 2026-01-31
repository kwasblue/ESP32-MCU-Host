// include/command/handlers/ControlHandler.h
// Handles control kernel commands: signals and controller slots

#pragma once

#include "command/ICommandHandler.h"
#include "command/CommandContext.h"
#include "module/ControlModule.h"
#include "control/SignalBus.h"
#include "core/Debug.h"
#include <algorithm>

class ControlHandler : public ICommandHandler {
public:
    ControlHandler() : controlModule_(nullptr) {}

    void setControlModule(ControlModule* cm) { controlModule_ = cm; }

    const char* name() const override { return "ControlHandler"; }

    bool canHandle(CmdType cmd) const override {
        switch (cmd) {
            // Signal commands
            case CmdType::CTRL_SIGNAL_DEFINE:
            case CmdType::CTRL_SIGNAL_SET:
            case CmdType::CTRL_SIGNAL_GET:
            case CmdType::CTRL_SIGNALS_LIST:
            case CmdType::CTRL_SIGNAL_DELETE:
            case CmdType::CTRL_SIGNALS_CLEAR:
            // Slot commands
            case CmdType::CTRL_SLOT_CONFIG:
            case CmdType::CTRL_SLOT_ENABLE:
            case CmdType::CTRL_SLOT_RESET:
            case CmdType::CTRL_SLOT_SET_PARAM:
            case CmdType::CTRL_SLOT_SET_PARAM_ARRAY:
            case CmdType::CTRL_SLOT_GET_PARAM:
            case CmdType::CTRL_SLOT_STATUS:
                return true;
            default:
                return false;
        }
    }

    void handle(CmdType cmd, JsonVariantConst payload, CommandContext& ctx) override {
        switch (cmd) {
            // Signals
            case CmdType::CTRL_SIGNAL_DEFINE:     handleSignalDefine(payload, ctx);     break;
            case CmdType::CTRL_SIGNAL_SET:        handleSignalSet(payload, ctx);        break;
            case CmdType::CTRL_SIGNAL_GET:        handleSignalGet(payload, ctx);        break;
            case CmdType::CTRL_SIGNALS_LIST:      handleSignalsList(ctx);               break;
            case CmdType::CTRL_SIGNAL_DELETE:     handleSignalDelete(payload, ctx);     break;
            case CmdType::CTRL_SIGNALS_CLEAR:     handleSignalsClear(ctx);              break;
            // Slots
            case CmdType::CTRL_SLOT_CONFIG:       handleSlotConfig(payload, ctx);       break;
            case CmdType::CTRL_SLOT_ENABLE:       handleSlotEnable(payload, ctx);       break;
            case CmdType::CTRL_SLOT_RESET:        handleSlotReset(payload, ctx);        break;
            case CmdType::CTRL_SLOT_SET_PARAM:    handleSlotSetParam(payload, ctx);     break;
            case CmdType::CTRL_SLOT_SET_PARAM_ARRAY: handleSlotSetParamArray(payload, ctx); break;
            case CmdType::CTRL_SLOT_GET_PARAM:    handleSlotGetParam(payload, ctx);     break;
            case CmdType::CTRL_SLOT_STATUS:       handleSlotStatus(payload, ctx);       break;
            default: break;
        }
    }

private:
    ControlModule* controlModule_;

    // -------------------------------------------------------------------------
    // Signal Commands
    // -------------------------------------------------------------------------

    void handleSignalDefine(JsonVariantConst payload, CommandContext& ctx) {
        static constexpr const char* ACK = "CMD_CTRL_SIGNAL_DEFINE";

        if (!ctx.requireIdle(ACK)) return;

        if (!controlModule_) {
            ctx.sendError(ACK, "no_control_module");
            return;
        }

        if (payload["id"].isNull() || payload["name"].isNull() || payload["kind"].isNull()) {
            ctx.sendError(ACK, "missing_fields");
            return;
        }

        uint16_t id = payload["id"].as<uint16_t>();
        const char* name = payload["name"].as<const char*>();
        const char* kindS = payload["signal_kind"] | payload["kind"] | "STATE";
        float initial = payload["initial"] | 0.0f;

        DBG_PRINTF("[CTRL] SIGNAL_DEFINE: id=%u name=%s kind=%s initial=%.2f\n",
                   id, name ? name : "null", kindS ? kindS : "null", initial);

        SignalBus::Kind kind = signalKindFromString(kindS);
        bool ok = controlModule_->signals().define(id, name, kind, initial);

        JsonDocument resp;
        resp["id"] = id;
        resp["name"] = name ? name : "";
        resp["kind"] = kindS ? kindS : "";
        resp["initial"] = initial;
        if (!ok) {
            resp["error"] = "define_failed";
        }
        ctx.sendAck(ACK, ok, resp);
    }

    void handleSignalSet(JsonVariantConst payload, CommandContext& ctx) {
        static constexpr const char* ACK = "CMD_CTRL_SIGNAL_SET";

        if (!controlModule_) {
            ctx.sendError(ACK, "no_control_module");
            return;
        }

        uint16_t id = payload["id"] | 0;
        float value = payload["value"] | 0.0f;

        bool ok = controlModule_->signals().set(id, value, millis());

        JsonDocument resp;
        resp["id"] = id;
        resp["value"] = value;
        if (!ok) {
            resp["error"] = "signal_not_found";
        }
        ctx.sendAck(ACK, ok, resp);
    }

    void handleSignalGet(JsonVariantConst payload, CommandContext& ctx) {
        static constexpr const char* ACK = "CMD_CTRL_SIGNAL_GET";

        if (!controlModule_) {
            ctx.sendError(ACK, "no_control_module");
            return;
        }

        uint16_t id = payload["id"] | 0;
        float value = 0.0f;
        bool ok = controlModule_->signals().get(id, value);

        JsonDocument resp;
        resp["id"] = id;
        resp["value"] = value;
        if (!ok) {
            resp["error"] = "signal_not_found";
        }
        ctx.sendAck(ACK, ok, resp);
    }

    void handleSignalsList(CommandContext& ctx) {
        static constexpr const char* ACK = "CMD_CTRL_SIGNALS_LIST";

        JsonDocument resp;

        if (!controlModule_) {
            resp["count"] = 0;
            resp["signals"].to<JsonArray>();
            ctx.sendAck(ACK, true, resp);
            return;
        }

        const auto& vec = controlModule_->signals().all();
        resp["count"] = static_cast<uint16_t>(vec.size());
        JsonArray arr = resp["signals"].to<JsonArray>();

        for (const auto& sdef : vec) {
            JsonObject s = arr.add<JsonObject>();
            s["id"] = sdef.id;
            s["name"] = sdef.name;
            s["kind"] = signalKindToString(sdef.kind);
            s["value"] = sdef.value;
            s["ts_ms"] = sdef.ts_ms;
        }

        ctx.sendAck(ACK, true, resp);
    }

    void handleSignalDelete(JsonVariantConst payload, CommandContext& ctx) {
        static constexpr const char* ACK = "CMD_CTRL_SIGNAL_DELETE";

        if (!controlModule_) {
            ctx.sendError(ACK, "no_control_module");
            return;
        }

        uint16_t id = payload["id"] | 0;
        bool ok = controlModule_->signals().remove(id);

        JsonDocument resp;
        resp["id"] = id;
        resp["deleted"] = ok;
        ctx.sendAck(ACK, ok, resp);
    }

    void handleSignalsClear(CommandContext& ctx) {
        static constexpr const char* ACK = "CMD_CTRL_SIGNALS_CLEAR";

        if (!controlModule_) {
            ctx.sendError(ACK, "no_control_module");
            return;
        }

        controlModule_->signals().clear();

        JsonDocument resp;
        resp["cleared"] = true;
        resp["count"] = 0;
        ctx.sendAck(ACK, true, resp);
    }

    // -------------------------------------------------------------------------
    // Slot Commands
    // -------------------------------------------------------------------------

    void handleSlotConfig(JsonVariantConst payload, CommandContext& ctx) {
        static constexpr const char* ACK = "CMD_CTRL_SLOT_CONFIG";

        if (!controlModule_) {
            ctx.sendError(ACK, "no_control_module");
            return;
        }

        SlotConfig cfg;
        cfg.slot = payload["slot"] | 0;
        cfg.rate_hz = payload["rate_hz"] | 100;
        cfg.require_armed = payload["require_armed"] | true;
        cfg.require_active = payload["require_active"] | true;

        const char* type = payload["controller_type"] | "PID";

        if (strcmp(type, "STATE_SPACE") == 0 || strcmp(type, "SS") == 0) {
            // State-space configuration
            cfg.ss_io.num_states = payload["num_states"] | 2;
            cfg.ss_io.num_inputs = payload["num_inputs"] | 1;

            JsonArrayConst state_ids = payload["state_ids"].as<JsonArrayConst>();
            if (state_ids) {
                for (size_t i = 0; i < state_ids.size() && i < StateSpaceIO::MAX_STATES; i++) {
                    cfg.ss_io.state_ids[i] = state_ids[i].as<uint16_t>();
                }
            }

            JsonArrayConst ref_ids = payload["ref_ids"].as<JsonArrayConst>();
            if (ref_ids) {
                for (size_t i = 0; i < ref_ids.size() && i < StateSpaceIO::MAX_STATES; i++) {
                    cfg.ss_io.ref_ids[i] = ref_ids[i].as<uint16_t>();
                }
            }

            JsonArrayConst out_ids = payload["output_ids"].as<JsonArrayConst>();
            if (out_ids) {
                for (size_t i = 0; i < out_ids.size() && i < StateSpaceIO::MAX_INPUTS; i++) {
                    cfg.ss_io.output_ids[i] = out_ids[i].as<uint16_t>();
                }
            }
        } else {
            // PID configuration
            cfg.io.ref_id = payload["ref_id"] | 0;
            cfg.io.meas_id = payload["meas_id"] | 0;
            cfg.io.out_id = payload["out_id"] | 0;
        }

        bool ok = controlModule_->kernel().configureSlot(cfg, type);

        JsonDocument resp;
        resp["slot"] = cfg.slot;
        resp["controller_type"] = type;
        resp["rate_hz"] = cfg.rate_hz;
        if (!ok) {
            resp["error"] = "config_failed";
        }
        ctx.sendAck(ACK, ok, resp);
    }

    void handleSlotEnable(JsonVariantConst payload, CommandContext& ctx) {
        static constexpr const char* ACK = "CMD_CTRL_SLOT_ENABLE";

        if (!controlModule_) {
            ctx.sendError(ACK, "no_control_module");
            return;
        }

        uint8_t slot = payload["slot"] | 0;
        bool enable = payload["enable"] | true;

        bool ok = controlModule_->kernel().enableSlot(slot, enable);

        JsonDocument resp;
        resp["slot"] = slot;
        resp["enable"] = enable;
        if (!ok) {
            resp["error"] = "enable_failed";
        }
        ctx.sendAck(ACK, ok, resp);
    }

    void handleSlotReset(JsonVariantConst payload, CommandContext& ctx) {
        static constexpr const char* ACK = "CMD_CTRL_SLOT_RESET";

        if (!controlModule_) {
            ctx.sendError(ACK, "no_control_module");
            return;
        }

        uint8_t slot = payload["slot"] | 0;
        bool ok = controlModule_->kernel().resetSlot(slot);

        JsonDocument resp;
        resp["slot"] = slot;
        if (!ok) {
            resp["error"] = "reset_failed";
        }
        ctx.sendAck(ACK, ok, resp);
    }

    void handleSlotSetParam(JsonVariantConst payload, CommandContext& ctx) {
        static constexpr const char* ACK = "CMD_CTRL_SLOT_SET_PARAM";

        if (!controlModule_) {
            ctx.sendError(ACK, "no_control_module");
            return;
        }

        uint8_t slot = payload["slot"] | 0;
        const char* key = payload["key"] | "";
        float value = payload["value"] | 0.0f;

        if (!key || key[0] == '\0') {
            ctx.sendError(ACK, "missing_key");
            return;
        }

        bool ok = controlModule_->kernel().setParam(slot, key, value);

        JsonDocument resp;
        resp["slot"] = slot;
        resp["key"] = key;
        resp["value"] = value;
        if (!ok) {
            resp["error"] = "set_param_failed";
        }
        ctx.sendAck(ACK, ok, resp);
    }

    void handleSlotSetParamArray(JsonVariantConst payload, CommandContext& ctx) {
        static constexpr const char* ACK = "CMD_CTRL_SLOT_SET_PARAM_ARRAY";

        if (!controlModule_) {
            ctx.sendError(ACK, "no_control_module");
            return;
        }

        uint8_t slot = payload["slot"] | 0;
        const char* key = payload["key"] | "";
        JsonArrayConst arr = payload["values"].as<JsonArrayConst>();

        if (!arr || arr.size() == 0) {
            JsonDocument resp;
            resp["slot"] = slot;
            resp["key"] = key;
            resp["error"] = "missing_values";
            ctx.sendAck(ACK, false, resp);
            return;
        }

        float values[12];  // Max size for K matrix (2x6)
        size_t len = std::min(arr.size(), sizeof(values) / sizeof(values[0]));
        for (size_t i = 0; i < len; i++) {
            values[i] = arr[i].as<float>();
        }

        bool ok = controlModule_->kernel().setParamArray(slot, key, values, len);

        JsonDocument resp;
        resp["slot"] = slot;
        resp["key"] = key;
        resp["count"] = (int)len;
        if (!ok) {
            resp["error"] = "set_param_array_failed";
        }
        ctx.sendAck(ACK, ok, resp);
    }

    void handleSlotGetParam(JsonVariantConst payload, CommandContext& ctx) {
        static constexpr const char* ACK = "CMD_CTRL_SLOT_GET_PARAM";

        if (!controlModule_) {
            ctx.sendError(ACK, "no_control_module");
            return;
        }

        uint8_t slot = payload["slot"] | 0;
        const char* key = payload["key"] | "";

        if (!key || key[0] == '\0') {
            ctx.sendError(ACK, "missing_key");
            return;
        }

        float value = 0.0f;
        bool ok = controlModule_->kernel().getParam(slot, key, value);

        JsonDocument resp;
        resp["slot"] = slot;
        resp["key"] = key;
        resp["value"] = value;
        if (!ok) {
            resp["error"] = "get_param_failed";
        }
        ctx.sendAck(ACK, ok, resp);
    }

    void handleSlotStatus(JsonVariantConst payload, CommandContext& ctx) {
        static constexpr const char* ACK = "CMD_CTRL_SLOT_STATUS";

        if (!controlModule_) {
            ctx.sendError(ACK, "no_control_module");
            return;
        }

        uint8_t slot = payload["slot"] | 0;

        auto cfg = controlModule_->kernel().getConfig(slot);
        auto st = controlModule_->kernel().getStatus(slot);

        JsonDocument resp;
        resp["slot"] = slot;
        resp["enabled"] = cfg.enabled;
        resp["rate_hz"] = cfg.rate_hz;
        resp["ok"] = st.ok;
        resp["run_count"] = st.run_count;
        resp["last_run_ms"] = st.last_run_ms;
        if (st.last_error) {
            resp["last_error"] = st.last_error;
        }
        ctx.sendAck(ACK, true, resp);
    }
};
