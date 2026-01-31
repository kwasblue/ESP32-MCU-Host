// include/command/handlers/MotionHandler.h
// Handles motion commands: SET_VEL, STOP

#pragma once

#include "command/ICommandHandler.h"
#include "command/CommandContext.h"
#include "motor/MotionController.h"
#include "core/Debug.h"
#include <cmath>

class MotionHandler : public ICommandHandler {
public:
    MotionHandler(MotionController& motion) : motion_(motion) {}

    const char* name() const override { return "MotionHandler"; }

    bool canHandle(CmdType cmd) const override {
        return cmd == CmdType::SET_VEL;
    }

    void handle(CmdType cmd, JsonVariantConst payload, CommandContext& ctx) override {
        if (cmd == CmdType::SET_VEL) {
            handleSetVel(payload, ctx);
        }
    }

private:
    MotionController& motion_;

    void handleSetVel(JsonVariantConst payload, CommandContext& ctx) {
        const uint32_t now_ms = millis();

        float vx = payload["vx"] | 0.0f;
        float omega = payload["omega"] | 0.0f;

        float safe_vx = 0.0f, safe_omega = 0.0f;
        if (!ctx.mode.validateVelocity(vx, omega, safe_vx, safe_omega)) {
            DBG_PRINTF("[MOTION] SET_VEL invalid: vx=%f omega=%f\n", vx, omega);
            ctx.sendError("CMD_SET_VEL", "invalid_velocity");
            return;
        }

        // Always allow "STOP" even if not armed
        const bool is_stop_cmd = (fabsf(safe_vx) < 1e-6f) && (fabsf(safe_omega) < 1e-6f);
        if (is_stop_cmd) {
            ctx.mode.onMotionCommand(now_ms);
            motion_.stop();

            JsonDocument resp;
            resp["vx"] = safe_vx;
            resp["omega"] = safe_omega;
            resp["state"] = robotModeToString(ctx.mode.mode());
            ctx.sendAck("CMD_SET_VEL", true, resp);
            return;
        }

        // Gate non-zero motion
        if (!ctx.mode.canMove()) {
            DBG_PRINTF("[MOTION] SET_VEL rejected: mode=%s\n", robotModeToString(ctx.mode.mode()));
            ctx.sendError("CMD_SET_VEL", "not_armed");
            return;
        }

        ctx.mode.onMotionCommand(now_ms);
        motion_.setVelocity(safe_vx, safe_omega);

        JsonDocument resp;
        resp["vx"] = safe_vx;
        resp["omega"] = safe_omega;
        resp["state"] = robotModeToString(ctx.mode.mode());
        ctx.sendAck("CMD_SET_VEL", true, resp);
    }
};
