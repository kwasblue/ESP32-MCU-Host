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
#include <algorithm>

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
        });
    }
}

void ControlModule::loop(uint32_t now_ms) {
    // Compute dt
    float dt_s = (last_step_ms_ > 0) 
        ? (now_ms - last_step_ms_) / 1000.0f 
        : 0.01f;
    
    // Clamp dt to reasonable range
    dt_s = std::max(0.001f, std::min(0.1f, dt_s));
    
    last_step_ms_ = now_ms;
    
    // Determine state flags
    bool is_armed = false;
    bool is_active = false;
    
    if (mode_) {
        is_armed = mode_->mode() >= RobotMode::ARMED;
        is_active = mode_->mode() == RobotMode::ACTIVE;
    }
    
    // Step observers FIRST (they provide state estimates to controllers)
    observers_.step(now_ms, dt_s, signals_);
    
    // Step controllers
    kernel_.step(now_ms, dt_s, signals_, is_armed, is_active);
}

void ControlModule::handleEvent(const Event& evt) {
    (void)evt;
    // No special event handling needed.
    // The kernel checks is_armed/is_active each step,
    // so ESTOP automatically stops all controllers.
}