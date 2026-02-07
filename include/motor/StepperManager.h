#pragma once

#include "config/FeatureFlags.h"

#if HAS_STEPPER

#include <Arduino.h>
#include <map>
#include "hw/GpioManager.h"

struct StepperConfig {
    int  pinStep;
    int  pinDir;
    int  pinEnable;   // -1 if unused
    bool invertDir;   // true if direction needs flipping
};

struct StepperState {
    StepperConfig cfg;

    // Logical GPIO channels in GpioManager
    int chStep   = -1;
    int chDir    = -1;
    int chEnable = -1;  // -1 if no enable

    bool attached       = false;
    bool enabled        = false;   // logical "driver enabled" state
    bool moving         = false;   // true while a blocking move is in progress
    bool lastDirForward = true;    // last commanded direction (after invertDir)
    int  lastCmdSteps   = 0;       // last commanded step count (signed)
    float lastCmdSpeed  = 0.0f;    // last commanded speed (steps/s)
};

class StepperManager {
public:
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

        bool enabled        = false;
        bool moving         = false;
        bool lastDirForward = true;
        int  lastCmdSteps   = 0;
        float lastCmdSpeed  = 0.0f;
    };

    explicit StepperManager(GpioManager& gpio)
        : gpio_(gpio) {}

    // Register a stepper motor
    void registerStepper(int motorId,
                         int pinStep,
                         int pinDir,
                         int pinEnable = -1,
                         bool invertDir = false);

    // Enable/disable motor driver
    void setEnabled(int motorId, bool enabled);

    // Blocking relative move with auto-enable/disable
    void moveRelative(int motorId, int steps, float speedStepsPerSec = 1000.0f);

    // Stop motor
    void stop(int motorId);

    // Get debug info for a motor
    bool getStepperDebugInfo(int motorId, StepperDebugInfo& out) const;

    // Dump all stepper mappings to debug output
    void dumpAllStepperMappings() const;

private:
    StepperState* getState(int motorId);
    const StepperState* getState(int motorId) const;

    GpioManager& gpio_;
    std::map<int, StepperState> steppers_;
};

#else // !HAS_STEPPER

class GpioManager;

// Stub when stepper is disabled
class StepperManager {
public:
    struct StepperDebugInfo {
        int motorId = -1;
        bool attached = false;
        int pinStep = -1;
        int pinDir = -1;
        int pinEnable = -1;
        bool invertDir = false;
        int chStep = -1;
        int chDir = -1;
        int chEnable = -1;
        bool enabled = false;
        bool moving = false;
        bool lastDirForward = true;
        int lastCmdSteps = 0;
        float lastCmdSpeed = 0.0f;
    };

    explicit StepperManager(GpioManager&) {}
    void registerStepper(int, int, int, int = -1, bool = false) {}
    void setEnabled(int, bool) {}
    void moveRelative(int, int, float = 1000.0f) {}
    void stop(int) {}
    bool getStepperDebugInfo(int, StepperDebugInfo&) const { return false; }
    void dumpAllStepperMappings() const {}
};

#endif // HAS_STEPPER
