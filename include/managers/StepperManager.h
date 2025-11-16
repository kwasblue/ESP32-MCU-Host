#pragma once
#include <Arduino.h>
#include <map>
#include "core/Debug.h"      // <-- add this

struct StepperConfig {
    int pinStep;
    int pinDir;
};

class StepperManager {
public:
    void registerStepper(int motorId, int pinStep, int pinDir) {
        pinMode(pinStep, OUTPUT);
        pinMode(pinDir, OUTPUT);

        steppers_[motorId] = { pinStep, pinDir };

        DBG_PRINTF("[STEPPER] registerStepper id=%d step=%d dir=%d\n",
                   motorId, pinStep, pinDir);
    }

    void moveRelative(int motorId, int steps, float speedStepsPerSec = 1000.0f) {
        if (!exists(motorId)) return;

        auto cfg = steppers_[motorId];

        digitalWrite(cfg.pinDir, (steps >= 0) ? HIGH : LOW);
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
        // No-op for open-loop motors, but kept for future
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
