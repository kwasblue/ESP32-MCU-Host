// src/core/ControlKernel.cpp
// Multi-slot controller manager implementation

#include "core/ControlKernel.h"
#include <cstring>
#include <algorithm>

// -----------------------------------------------------------------------------
// ControlKernel Implementation
// -----------------------------------------------------------------------------

ControlKernel::Slot* ControlKernel::getSlot_(uint8_t slot) {
    for (auto& s : slots_) {
        if (s.cfg.slot == slot) return &s;
    }
    return nullptr;
}

const ControlKernel::Slot* ControlKernel::getSlot_(uint8_t slot) const {
    for (const auto& s : slots_) {
        if (s.cfg.slot == slot) return &s;
    }
    return nullptr;
}

void ControlKernel::ensureSlot_(uint8_t slot) {
    if (getSlot_(slot)) return;
    if (slots_.size() >= MAX_SLOTS) return;
    
    Slot s;
    s.cfg.slot = slot;
    slots_.push_back(std::move(s));
}

bool ControlKernel::configureSlot(const SlotConfig& cfg, const char* type) {
    if (cfg.slot >= MAX_SLOTS) return false;
    
    ensureSlot_(cfg.slot);
    Slot* s = getSlot_(cfg.slot);
    if (!s) return false;
    
    s->cfg = cfg;
    s->cfg.enabled = false;  // Start disabled
    s->status = SlotStatus{};
    
    // Create controller based on type
    if (strcmp(type, "PID") == 0) {
        s->ctrl = std::make_unique<PidController>();
    } else {
        // Unknown type
        s->ctrl.reset();
        return false;
    }
    
    return true;
}

bool ControlKernel::enableSlot(uint8_t slot, bool enable) {
    Slot* s = getSlot_(slot);
    if (!s || !s->ctrl) return false;
    
    s->cfg.enabled = enable;
    if (!enable) {
        s->ctrl->reset();
    }
    return true;
}

bool ControlKernel::resetSlot(uint8_t slot) {
    Slot* s = getSlot_(slot);
    if (!s || !s->ctrl) return false;
    
    s->ctrl->reset();
    s->status.last_error = nullptr;
    return true;
}

bool ControlKernel::setParam(uint8_t slot, const char* key, float value) {
    Slot* s = getSlot_(slot);
    if (!s || !s->ctrl) return false;
    
    return s->ctrl->setParam(key, value);
}

SlotConfig ControlKernel::getConfig(uint8_t slot) const {
    const Slot* s = getSlot_(slot);
    if (!s) {
        SlotConfig empty;
        empty.slot = slot;
        return empty;
    }
    return s->cfg;
}

SlotStatus ControlKernel::getStatus(uint8_t slot) const {
    const Slot* s = getSlot_(slot);
    if (!s) return SlotStatus{};
    return s->status;
}

void ControlKernel::step(uint32_t now_ms, float dt_s, SignalBus& signals, bool is_armed, bool is_active) {
    for (auto& s : slots_) {
        if (!s.cfg.enabled || !s.ctrl) continue;
        
        // Check state gating
        if (s.cfg.require_armed && !is_armed) continue;
        if (s.cfg.require_active && !is_active) continue;
        
        // Check rate limiting
        uint32_t period_ms = 1000 / s.cfg.rate_hz;
        if (now_ms - s.last_step_ms < period_ms) continue;
        s.last_step_ms = now_ms;
        
        // Get signals
        float ref = 0.0f, meas = 0.0f;
        if (!signals.get(s.cfg.io.ref_id, ref)) {
            s.status.last_error = "ref_signal_missing";
            s.status.ok = false;
            continue;
        }
        if (!signals.get(s.cfg.io.meas_id, meas)) {
            s.status.last_error = "meas_signal_missing";
            s.status.ok = false;
            continue;
        }
        
        // Compute control output
        float out = s.ctrl->compute(ref, meas, dt_s);
        
        // Write output signal
        signals.set(s.cfg.io.out_id, out, now_ms);
        
        // Update status
        s.status.ok = true;
        s.status.run_count++;
        s.status.last_run_ms = now_ms;
        s.status.last_error = nullptr;
    }
}

void ControlKernel::resetAll() {
    for (auto& s : slots_) {
        if (s.ctrl) s.ctrl->reset();
        s.status = SlotStatus{};
    }
}

void ControlKernel::disableAll() {
    for (auto& s : slots_) {
        s.cfg.enabled = false;
        if (s.ctrl) s.ctrl->reset();
    }
}

// -----------------------------------------------------------------------------
// PidController Implementation
// -----------------------------------------------------------------------------

PidController::PidController(float kp, float ki, float kd)
    : kp_(kp), ki_(ki), kd_(kd) {}

float PidController::compute(float ref, float meas, float dt_s) {
    float error = ref - meas;
    
    // Proportional
    float p_term = kp_ * error;
    
    // Integral
    if (dt_s > 0.0f) {
        integral_ += error * dt_s;
        integral_ = std::max(i_min_, std::min(i_max_, integral_));
    }
    float i_term = ki_ * integral_;
    
    // Derivative
    float d_term = 0.0f;
    if (!first_run_ && dt_s > 0.0f) {
        d_term = kd_ * (error - prev_error_) / dt_s;
    }
    prev_error_ = error;
    first_run_ = false;
    
    // Sum and clamp
    float out = p_term + i_term + d_term;
    out = std::max(out_min_, std::min(out_max_, out));
    
    return out;
}

void PidController::reset() {
    integral_ = 0.0f;
    prev_error_ = 0.0f;
    first_run_ = true;
}

bool PidController::setParam(const char* key, float value) {
    if (!key) return false;
    
    if (strcmp(key, "kp") == 0) { kp_ = value; return true; }
    if (strcmp(key, "ki") == 0) { ki_ = value; return true; }
    if (strcmp(key, "kd") == 0) { kd_ = value; return true; }
    if (strcmp(key, "out_min") == 0) { out_min_ = value; return true; }
    if (strcmp(key, "out_max") == 0) { out_max_ = value; return true; }
    if (strcmp(key, "i_min") == 0) { i_min_ = value; return true; }
    if (strcmp(key, "i_max") == 0) { i_max_ = value; return true; }
    
    return false;
}

bool PidController::getParam(const char* key, float& value) const {
    if (!key) return false;
    
    if (strcmp(key, "kp") == 0) { value = kp_; return true; }
    if (strcmp(key, "ki") == 0) { value = ki_; return true; }
    if (strcmp(key, "kd") == 0) { value = kd_; return true; }
    if (strcmp(key, "out_min") == 0) { value = out_min_; return true; }
    if (strcmp(key, "out_max") == 0) { value = out_max_; return true; }
    if (strcmp(key, "i_min") == 0) { value = i_min_; return true; }
    if (strcmp(key, "i_max") == 0) { value = i_max_; return true; }
    
    return false;
}

void PidController::setGains(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PidController::setOutputLimits(float min_out, float max_out) {
    out_min_ = min_out;
    out_max_ = max_out;
}

void PidController::setIntegralLimits(float min_i, float max_i) {
    i_min_ = min_i;
    i_max_ = max_i;
}