#pragma once
#include <Arduino.h>
#include <map>

struct StepperConfig {
    int pinStep;
    int pinDir;
};

class StepperManager {
public:
    void registerStepper(int motorId, int pinStep, int pinDir) {
        pinMode(pinStep, OUTPUT);
        pinMode(pinDir, OUTPUT);
        steppers_[motorId] = {pinStep, pinDir};
    }

    void moveRelative(int motorId, int steps, float speedStepsPerSec=1000.0f) {
        if (!exists(motorId)) return;

        auto cfg = steppers_[motorId];

        digitalWrite(cfg.pinDir, (steps >= 0) ? HIGH : LOW);
        int count = abs(steps);

        int delayMicros = (int)(1000000.0f / speedStepsPerSec / 2.0f);

        for (int i = 0; i < count; i++) {
            digitalWrite(cfg.pinStep, HIGH);
            delayMicroseconds(delayMicros);
            digitalWrite(cfg.pinStep, LOW);
            delayMicroseconds(delayMicros);
        }
    }

    void stop(int motorId) {
        // No real "stop" for open-loop; but used for future features
    }

private:
    bool exists(int motorId) {
        if (!steppers_.count(motorId)) {
            Serial.printf("[STEPPER] Unknown motor id=%d\n", motorId);
            return false;
        }
        return true;
    }

    std::map<int, StepperConfig> steppers_;
};
