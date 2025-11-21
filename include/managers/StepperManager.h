// include/managers/StepperManager.h
#pragma once

#include <Arduino.h>
#include <map>
#include "core/Debug.h"
#include "managers/GpioManager.h"

struct StepperConfig {
    int  pinStep;
    int  pinDir;
    int  pinEnable;   // -1 if unused
    bool invertDir;   // true if direction needs flipping
};

// Internal expanded state
struct StepperState {
    StepperConfig cfg;

    // Logical GPIO channels in GpioManager
    int chStep   = -1;
    int chDir    = -1;
    int chEnable = -1;  // -1 if no enable

    bool attached = false;
};

class StepperManager {
public:
    // Public debug view
    struct StepperDebugInfo {
        int  motorId     = -1;
        bool attached    = false;

        int  pinStep     = -1;
        int  pinDir      = -1;
        int  pinEnable   = -1;
        bool invertDir   = false;

        int  chStep      = -1;
        int  chDir       = -1;
        int  chEnable    = -1;
    };

    explicit StepperManager(GpioManager& gpio)
        : gpio_(gpio) {}

    void registerStepper(int motorId,
                         int pinStep,
                         int pinDir,
                         int pinEnable = -1,
                         bool invertDir = false)
    {
        // Derive logical channels from motorId.
        // 3 channels per stepper: step, dir, enable
        const int baseCh   = motorId * 3;
        const int chStep   = baseCh;
        const int chDir    = baseCh + 1;
        const int chEnable = (pinEnable >= 0) ? (baseCh + 2) : -1;

        // Configure pins via GpioManager
        gpio_.registerChannel(chStep, pinStep, OUTPUT);
        gpio_.registerChannel(chDir,  pinDir,  OUTPUT);

        if (chEnable >= 0) {
            gpio_.registerChannel(chEnable, pinEnable, OUTPUT);
            gpio_.write(chEnable, LOW);  // enable by default (typical A4988 logic)
        }

        StepperState st;
        st.cfg.pinStep   = pinStep;
        st.cfg.pinDir    = pinDir;
        st.cfg.pinEnable = pinEnable;
        st.cfg.invertDir = invertDir;

        st.chStep        = chStep;
        st.chDir         = chDir;
        st.chEnable      = chEnable;
        st.attached      = true;

        steppers_[motorId] = st;

        DBG_PRINTF("[STEPPER] registerStepper id=%d stepPin=%d dirPin=%d enPin=%d "
                   "chStep=%d chDir=%d chEn=%d invert=%d\n",
                   motorId, pinStep, pinDir, pinEnable,
                   chStep, chDir, chEnable, invertDir);
    }

    void setEnabled(int motorId, bool enabled) {
        StepperState* st = getState(motorId);
        if (!st) return;

        if (st->chEnable < 0) {
            return;  // no enable pin
        }

        // Typical A4988: LOW = enabled, HIGH = disabled
        gpio_.write(st->chEnable, enabled ? LOW : HIGH);

        DBG_PRINTF("[STEPPER] setEnabled id=%d -> %d\n", motorId, enabled);
    }

    // Blocking relative move, same semantics as your original
    void moveRelative(int motorId, int steps, float speedStepsPerSec = 1000.0f) {
        StepperState* st = getState(motorId);
        if (!st) return;

        bool dirForward = (steps >= 0);
        if (st->cfg.invertDir) {
            dirForward = !dirForward;
        }

        gpio_.write(st->chDir, dirForward ? HIGH : LOW);

        int count = abs(steps);
        if (count == 0 || speedStepsPerSec <= 0.0f) {
            DBG_PRINTF("[STEPPER] moveRelative id=%d NOOP (steps=%d speed=%.1f)\n",
                       motorId, steps, speedStepsPerSec);
            return;
        }

        int delayMicros = (int)(1000000.0f / speedStepsPerSec / 2.0f);

        DBG_PRINTF("[STEPPER] moveRelative id=%d steps=%d speed=%.1f delay=%dus\n",
                   motorId, steps, speedStepsPerSec, delayMicros);

        for (int i = 0; i < count; ++i) {
            gpio_.write(st->chStep, HIGH);
            delayMicroseconds(delayMicros);
            gpio_.write(st->chStep, LOW);
            delayMicroseconds(delayMicros);
        }

        DBG_PRINTF("[STEPPER] moveRelative id=%d DONE (%d steps)\n",
                   motorId, steps);
    }

    void stop(int motorId) {
        // For now, noop. Later, you can add ramp-down or non-blocking handling.
        DBG_PRINTF("[STEPPER] stop id=%d (noop)\n", motorId);
    }

    // === Debug helpers ===

    bool getStepperDebugInfo(int motorId, StepperDebugInfo& out) const {
        auto it = steppers_.find(motorId);
        if (it == steppers_.end()) {
            return false;
        }
        const StepperState& st = it->second;

        out.motorId   = motorId;
        out.attached  = st.attached;
        out.pinStep   = st.cfg.pinStep;
        out.pinDir    = st.cfg.pinDir;
        out.pinEnable = st.cfg.pinEnable;
        out.invertDir = st.cfg.invertDir;
        out.chStep    = st.chStep;
        out.chDir     = st.chDir;
        out.chEnable  = st.chEnable;

        return true;
    }

    void dumpAllStepperMappings() const {
        DBG_PRINTF("=== StepperManager mappings ===\n");
        if (steppers_.empty()) {
            DBG_PRINTF("  [no steppers registered]\n");
        }

        for (const auto& kv : steppers_) {
            int motorId = kv.first;
            const StepperState& st = kv.second;
            DBG_PRINTF(
                "  id=%d attached=%d stepPin=%d dirPin=%d enPin=%d "
                "chStep=%d chDir=%d chEn=%d invert=%d\n",
                motorId,
                st.attached,
                st.cfg.pinStep,
                st.cfg.pinDir,
                st.cfg.pinEnable,
                st.chStep,
                st.chDir,
                st.chEnable,
                st.cfg.invertDir
            );
        }
        DBG_PRINTF("=== end StepperManager mappings ===\n");
    }

private:
    StepperState* getState(int motorId) {
        auto it = steppers_.find(motorId);
        if (it == steppers_.end()) {
            DBG_PRINTF("[STEPPER] Unknown motor id=%d\n", motorId);
            return nullptr;
        }
        if (!it->second.attached) {
            DBG_PRINTF("[STEPPER] motor id=%d not attached\n", motorId);
            return nullptr;
        }
        return &it->second;
    }

    const StepperState* getState(int motorId) const {
        auto it = steppers_.find(motorId);
        if (it == steppers_.end() || !it->second.attached) {
            return nullptr;
        }
        return &it->second;
    }

    GpioManager& gpio_;
    std::map<int, StepperState> steppers_;
};
