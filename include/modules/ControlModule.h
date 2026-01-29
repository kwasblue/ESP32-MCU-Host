// include/modules/ControlModule.h
// Control system module - manages signal bus and control kernel

#pragma once

#include "core/IModule.h"
#include "core/SignalBus.h"
#include "core/ControlKernel.h"

// Forward declarations
class EventBus;
class ModeManager;
class MotionController;
class EncoderManager;
class ImuManager;
class TelemetryModule;

class ControlModule : public IModule {
public:
    ControlModule(
        EventBus* bus,
        ModeManager* mode,
        MotionController* motion,
        EncoderManager* encoder,
        ImuManager* imu,
        TelemetryModule* telemetry
    );

    // IModule interface
    void setup() override;
    void loop(uint32_t now_ms) override;
    void handleEvent(const Event& evt) override;
    
    // Module name (not in base class, so no override)
    const char* name() const { return "ControlModule"; }

    // Access to signal bus and kernel
    SignalBus& signals() { return signals_; }
    const SignalBus& signals() const { return signals_; }
    
    ControlKernel& kernel() { return kernel_; }
    const ControlKernel& kernel() const { return kernel_; }

private:
    EventBus* bus_ = nullptr;
    ModeManager* mode_ = nullptr;
    MotionController* motion_ = nullptr;
    EncoderManager* encoder_ = nullptr;
    ImuManager* imu_ = nullptr;
    TelemetryModule* telemetry_ = nullptr;

    SignalBus signals_;
    ControlKernel kernel_;
    
    uint32_t last_step_ms_ = 0;
    
    // Sample sensors into signals
    void sampleSensors(uint32_t now_ms);
    
    // Apply output signals to actuators
    void applyOutputs(uint32_t now_ms);
};