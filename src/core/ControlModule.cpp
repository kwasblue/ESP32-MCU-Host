// src/modules/ControlModule.cpp
// Control system module implementation

#include "modules/ControlModule.h"
#include "core/EventBus.h"
#include "core/ModeManager.h"
#include "core/MotionController.h"
#include "core/LoopRates.h"
#include "managers/EncoderManager.h"
#include "managers/ImuManager.h"
#include "modules/TelemetryModule.h"

ControlModule::ControlModule(
    EventBus* bus,
    ModeManager* mode,
    MotionController* motion,
    EncoderManager* encoder,
    ImuManager* imu,
    TelemetryModule* telemetry
)
    : bus_(bus)
    , mode_(mode)
    , motion_(motion)
    , encoder_(encoder)
    , imu_(imu)
    , telemetry_(telemetry)
{
}

void ControlModule::setup() {
    // Register telemetry provider for control kernel status
    if (telemetry_) {
        telemetry_->registerProvider("control", [this](ArduinoJson::JsonObject node) {
            node["signals"] = static_cast<uint16_t>(signals_.count());
            
            // Could add more detailed status here
        });
    }
}

void ControlModule::loop(uint32_t now_ms) {
    // The control kernel is stepped via commands or can be integrated here later.
    // For now, keep this minimal to avoid conflicts with main.cpp's runControlLoop().
    // 
    // If you want the kernel to auto-step, uncomment the code below.
    // But ensure you're not double-running control logic.
    
    (void)now_ms;
    
    // Uncomment below to enable automatic kernel stepping:
    /*
    if (!mode_) return;
    
    // Get control loop period from global rates
    LoopRates& rates = getLoopRates();
    uint32_t period_ms = rates.ctrl_period_ms();
    
    // Rate limit control loop
    if (now_ms - last_step_ms_ < period_ms) {
        return;
    }
    
    float dt_s = (now_ms - last_step_ms_) / 1000.0f;
    last_step_ms_ = now_ms;
    
    // Sample sensors into signal bus
    sampleSensors(now_ms);
    
    // Determine state flags
    bool is_armed = mode_->canMove();
    bool is_active = (mode_->mode() == RobotMode::ACTIVE);
    
    // Step all controllers
    kernel_.step(now_ms, dt_s, signals_, is_armed, is_active);
    
    // Apply outputs to actuators
    applyOutputs(now_ms);
    */
}

void ControlModule::handleEvent(const Event& evt) {
    (void)evt;
    // No special event handling needed.
    // The kernel already checks is_armed/is_active each step,
    // so ESTOP automatically stops all controllers.
}

void ControlModule::sampleSensors(uint32_t now_ms) {
    // Example: Sample encoder 0 into a measurement signal
    // This would be configured by the user defining signals and slots
    
    // For now, this is a placeholder. Users define signals via commands
    // and can set up their own sensor sampling outside this module,
    // or we can add auto-sampling configuration later.
    
    (void)now_ms;
}

void ControlModule::applyOutputs(uint32_t now_ms) {
    // Example: Read output signals and apply to motors
    // This would be configured by the user
    
    // For now, this is a placeholder. Users can read output signals
    // via commands or telemetry and apply them externally,
    // or we can add auto-application configuration later.
    
    (void)now_ms;
}