#pragma once
#include <Arduino.h>
#include <map>
#include "core/Debug.h"      // <-- add this

struct StepperConfig {
    int pinStep;
    int pinDir;
    int pinEnable;   // -1 if unused
    bool invertDir;  // true if direction needs flipping
};

class StepperManager {
public:
    void registerStepper(int motorId,
                         int pinStep,
                         int pinDir,
                         int pinEnable = -1,
                         bool invertDir = false)
    {
        pinMode(pinStep, OUTPUT);
        pinMode(pinDir, OUTPUT);

        if (pinEnable >= 0) {
            pinMode(pinEnable, OUTPUT);
            digitalWrite(pinEnable, LOW);  // enable by default
        }

        steppers_[motorId] = { pinStep, pinDir, pinEnable, invertDir };

        DBG_PRINTF("[STEPPER] registerStepper id=%d step=%d dir=%d en=%d inv=%d\n",
                   motorId, pinStep, pinDir, pinEnable, invertDir);
    }

    void setEnabled(int motorId, bool enabled) {
        if (!exists(motorId)) return;
        auto cfg = steppers_[motorId];
        if (cfg.pinEnable < 0) return;

        digitalWrite(cfg.pinEnable, enabled ? LOW : HIGH); // typical A4988 logic
        DBG_PRINTF("[STEPPER] setEnabled id=%d -> %d\n", motorId, enabled);
    }

    void moveRelative(int motorId, int steps, float speedStepsPerSec = 1000.0f) {
        if (!exists(motorId)) return;

        auto cfg = steppers_[motorId];

        bool dirForward = (steps >= 0);
        if (cfg.invertDir) {
            dirForward = !dirForward;
        }
        digitalWrite(cfg.pinDir, dirForward ? HIGH : LOW);

        int count = abs(steps);
        int delayMicros = (int)(1000000.0f / speedStepsPerSec / 2.0f);

        DBG_PRINTF("[STEPPER] moveRelative id=%d steps=%d speed=%.1f delay=%dus\n",
                   motorId, steps, speedStepsPerSec, delayMicros);

        for (int i = 0; i < count; i++) {
            digitalWrite(cfg.pinStep, HIGH);
            delayMicroseconds(delayMicros);
            digitalWrite(cfg.pinStep, LOW);
            delayMicroseconds(delayMicros);
        }

        DBG_PRINTF("[STEPPER] moveRelative id=%d DONE (%d steps)\n",
                   motorId, steps);
    }

    void stop(int motorId) {
        // For now, noop. Later you can add ramp-down if you go non-blocking.
        DBG_PRINTF("[STEPPER] stop id=%d (noop)\n", motorId);
    }

private:
    bool exists(int motorId) {
        if (!steppers_.count(motorId)) {
            DBG_PRINTF("[STEPPER] Unknown motor id=%d\n", motorId);
            return false;
        }
        return true;
    }

    std::map<int, StepperConfig> steppers_;
};
