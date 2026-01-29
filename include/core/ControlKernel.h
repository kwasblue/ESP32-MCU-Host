// include/core/ControlKernel.h
// Multi-slot controller manager for real-time control

#pragma once

#include <cstdint>
#include <vector>
#include <memory>
#include "core/SignalBus.h"

// Forward declaration
class IController;

// Slot I/O configuration
struct SlotIO {
    uint16_t ref_id  = 0;   // Reference/setpoint signal ID
    uint16_t meas_id = 0;   // Measurement signal ID
    uint16_t out_id  = 0;   // Output signal ID
};

// Slot configuration
struct SlotConfig {
    uint8_t slot = 0;
    uint16_t rate_hz = 100;
    bool enabled = false;
    SlotIO io;
    bool require_armed = true;
    bool require_active = true;
};

// Slot runtime status
struct SlotStatus {
    bool ok = false;
    uint32_t run_count = 0;
    uint32_t last_run_ms = 0;
    const char* last_error = nullptr;
};

class ControlKernel {
public:
    static constexpr size_t MAX_SLOTS = 8;

    ControlKernel() = default;
    ~ControlKernel() = default;

    // Configure a slot with controller type ("PID" or "SS")
    bool configureSlot(const SlotConfig& cfg, const char* type);
    
    // Enable/disable a slot
    bool enableSlot(uint8_t slot, bool enable);
    
    // Reset a slot's controller state
    bool resetSlot(uint8_t slot);
    
    // Set a parameter on a slot's controller
    bool setParam(uint8_t slot, const char* key, float value);
    
    // Get slot configuration
    SlotConfig getConfig(uint8_t slot) const;
    
    // Get slot status
    SlotStatus getStatus(uint8_t slot) const;
    
    // Step all enabled controllers
    void step(uint32_t now_ms, float dt_s, SignalBus& signals, bool is_armed, bool is_active);
    
    // Reset all slots
    void resetAll();
    
    // Disable all slots
    void disableAll();

private:
    struct Slot {
        SlotConfig cfg;
        SlotStatus status;
        std::unique_ptr<IController> ctrl;
        uint32_t last_step_ms = 0;
    };

    std::vector<Slot> slots_;
    
    Slot* getSlot_(uint8_t slot);
    const Slot* getSlot_(uint8_t slot) const;
    void ensureSlot_(uint8_t slot);
};

// -----------------------------------------------------------------------------
// IController Interface
// -----------------------------------------------------------------------------
class IController {
public:
    virtual ~IController() = default;
    
    // Compute output from error (ref - meas)
    virtual float compute(float ref, float meas, float dt_s) = 0;
    
    // Reset internal state (integrators, etc)
    virtual void reset() = 0;
    
    // Set a parameter by name
    virtual bool setParam(const char* key, float value) = 0;
    
    // Get a parameter by name
    virtual bool getParam(const char* key, float& value) const = 0;
};

// -----------------------------------------------------------------------------
// PID Controller Implementation
// -----------------------------------------------------------------------------
class PidController : public IController {
public:
    PidController(float kp = 1.0f, float ki = 0.0f, float kd = 0.0f);
    
    float compute(float ref, float meas, float dt_s) override;
    void reset() override;
    bool setParam(const char* key, float value) override;
    bool getParam(const char* key, float& value) const override;
    
    // Direct setters
    void setGains(float kp, float ki, float kd);
    void setOutputLimits(float min_out, float max_out);
    void setIntegralLimits(float min_i, float max_i);

private:
    float kp_ = 1.0f;
    float ki_ = 0.0f;
    float kd_ = 0.0f;
    
    float out_min_ = -1.0f;
    float out_max_ = 1.0f;
    float i_min_ = -1.0f;
    float i_max_ = 1.0f;
    
    float integral_ = 0.0f;
    float prev_error_ = 0.0f;
    bool first_run_ = true;
};