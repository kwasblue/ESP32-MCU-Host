// src/core/ModeManager.cpp

#include "command/ModeManager.h"
#include "core/CriticalSection.h"
#include <Arduino.h>
#include <cmath>

const char* robotModeToString(RobotMode m) {
    switch (m) {
        case RobotMode::BOOT:         return "BOOT";
        case RobotMode::DISCONNECTED: return "DISCONNECTED";
        case RobotMode::IDLE:         return "IDLE";
        case RobotMode::ARMED:        return "ARMED";
        case RobotMode::ACTIVE:       return "ACTIVE";
        case RobotMode::ESTOPPED:     return "ESTOPPED";
        default:                      return "UNKNOWN";
    }
}

void ModeManager::begin() {
    // Watchdog timeout: 2 seconds (was 5s - reduced for faster fault detection)
    // If the main loop hangs for >2s, MCU will reset
    if (halWatchdog_) {
        halWatchdog_->begin(2, true);
        halWatchdog_->addCurrentTask();
    }

    // Configure safety pins via HAL
    if (halGpio_) {
        if (cfg_.estop_pin >= 0) {
            halGpio_->pinMode(cfg_.estop_pin, hal::PinMode::InputPullup);
        }
        if (cfg_.bypass_pin >= 0) {
            halGpio_->pinMode(cfg_.bypass_pin, hal::PinMode::InputPullup);
        }
        if (cfg_.relay_pin >= 0) {
            halGpio_->pinMode(cfg_.relay_pin, hal::PinMode::Output);
            halGpio_->digitalWrite(cfg_.relay_pin, 0);
        }
    }

    mode_ = RobotMode::DISCONNECTED;
}

void ModeManager::update(uint32_t now_ms) {
    if (halWatchdog_) {
        halWatchdog_->reset();
    }
    readHardwareInputs();

    // Host timeout
    if (hostEverSeen_ && isConnected() && !isEstopped()) {
        uint32_t dt = now_ms - lastHostHeartbeat_;
        if (dt > cfg_.host_timeout_ms) {
            mcu::CriticalSection lock(lock_);
            triggerStop();

            if (mode_ == RobotMode::ACTIVE) {
                mode_ = RobotMode::ARMED;     // fall back from ACTIVE
            } else if (mode_ == RobotMode::ARMED) {
                mode_ = RobotMode::ARMED;     // stay ARMED (don't disarm)
            } else {
                mode_ = RobotMode::IDLE;
            }

            lastHostHeartbeat_ = now_ms;      // prevent retrigger spam
        }
    }

    // Motion timeout (only in ACTIVE and only if robot was actually moving)
    // This prevents timeout spam during testing when no motion is commanded
    if (mode_ == RobotMode::ACTIVE && lastMotionCmd_ > 0 && wasMoving_) {
        uint32_t dtm = now_ms - lastMotionCmd_;
        if (dtm > cfg_.motion_timeout_ms) {
            mcu::CriticalSection lock(lock_);
            triggerStop();
            mode_ = RobotMode::ARMED;
            wasMoving_ = false;

            // Prevent re-triggering every loop iteration
            lastMotionCmd_ = now_ms;
        }
    }
    if (mode_ != lastLoggedMode_) {
        Serial.printf("[MODE] %s -> %s  hostAge=%lu  motionAge=%lu\n",
            robotModeToString(lastLoggedMode_),
            robotModeToString(mode_),
            (unsigned long)hostAgeMs(now_ms),
            (unsigned long)motionAgeMs(now_ms)
        );
        lastLoggedMode_ = mode_;
    }

    // Relay control via HAL
    if (cfg_.relay_pin >= 0 && halGpio_) {
        bool allow = canMove() && !isEstopped();
        halGpio_->digitalWrite(cfg_.relay_pin, allow ? 1 : 0);
    }
}

void ModeManager::readHardwareInputs() {
    if (!halGpio_) return;

    if (cfg_.bypass_pin >= 0) {
        bypassed_ = (halGpio_->digitalRead(cfg_.bypass_pin) == 0);
    }
    if (cfg_.estop_pin >= 0) {
        if (halGpio_->digitalRead(cfg_.estop_pin) == 0 && !isEstopped()) {
            estop();
        }
    }
}

void ModeManager::onHostHeartbeat(uint32_t now_ms) {
    lastHostHeartbeat_ = now_ms;

    if (!hostEverSeen_) {
        hostEverSeen_ = true;
    }

    if (mode_ == RobotMode::DISCONNECTED) {
        mode_ = RobotMode::IDLE;
    }
}

void ModeManager::onMotionCommand(uint32_t now_ms, float vx, float omega) {
    lastMotionCmd_ = now_ms;
    // Track if actual motion was commanded (non-zero velocity)
    // Motion timeout only applies if robot was actually moving
    if (fabsf(vx) > 0.001f || fabsf(omega) > 0.001f) {
        wasMoving_ = true;
    }
}

bool ModeManager::canTransition(RobotMode from, RobotMode to) {
    switch (from) {
        case RobotMode::BOOT:
            return to == RobotMode::DISCONNECTED;
        case RobotMode::DISCONNECTED:
            return to == RobotMode::IDLE || to == RobotMode::ESTOPPED;
        case RobotMode::IDLE:
            return to == RobotMode::ARMED || to == RobotMode::DISCONNECTED || to == RobotMode::ESTOPPED;
        case RobotMode::ARMED:
            return to == RobotMode::IDLE || to == RobotMode::ACTIVE || to == RobotMode::DISCONNECTED || to == RobotMode::ESTOPPED;
        case RobotMode::ACTIVE:
            return to == RobotMode::ARMED || to == RobotMode::DISCONNECTED || to == RobotMode::ESTOPPED;
        case RobotMode::ESTOPPED:
            return to == RobotMode::IDLE;
        default:
            return false;
    }
}

void ModeManager::arm() {
    mcu::CriticalSection lock(lock_);
    if (mode_ == RobotMode::IDLE) {
        mode_ = RobotMode::ARMED;
    }
}

void ModeManager::activate() {
    mcu::CriticalSection lock(lock_);
    if (mode_ == RobotMode::ARMED) {
        lastMotionCmd_ = now_ms();
        mode_ = RobotMode::ACTIVE;
    }
}

void ModeManager::deactivate() {
    mcu::CriticalSection lock(lock_);
    if (mode_ == RobotMode::ACTIVE) {
        triggerStop();
        mode_ = RobotMode::ARMED;
        lastMotionCmd_ = now_ms();
    }
}

void ModeManager::disarm() {
    mcu::CriticalSection lock(lock_);
    if (mode_ == RobotMode::ARMED || mode_ == RobotMode::ACTIVE) {
        triggerStop();
        mode_ = RobotMode::IDLE;
    }
}

void ModeManager::estop() {
    mcu::CriticalSection lock(lock_);
    // Emergency stop: trigger both normal and emergency callbacks
    triggerStop();
    triggerEmergencyStop();  // Direct motor disable - doesn't rely on motion controller
    mode_ = RobotMode::ESTOPPED;
}

bool ModeManager::clearEstop() {
    mcu::CriticalSection lock(lock_);
    if (!isEstopped()) {
        return true;
    }

    // Hardware E-stop must be released
    if (cfg_.estop_pin >= 0 && halGpio_) {
        if (halGpio_->digitalRead(cfg_.estop_pin) == 0) {
            return false;
        }
    }

    mode_ = RobotMode::IDLE;
    return true;
}

bool ModeManager::validateVelocity(float vx, float omega, float& out_vx, float& out_omega) {
    if (std::isnan(vx) || std::isinf(vx) || std::isnan(omega) || std::isinf(omega)) {
        out_vx = 0;
        out_omega = 0;
        return false;
    }

    out_vx = constrain(vx, -cfg_.max_linear_vel, cfg_.max_linear_vel);
    out_omega = constrain(omega, -cfg_.max_angular_vel, cfg_.max_angular_vel);
    return true;
}

void ModeManager::triggerStop() {
    if (bypassed_) return;
    if (stopCallback_) stopCallback_();
}

void ModeManager::triggerEmergencyStop() {
    // Emergency stop bypasses the bypass flag - always stops motors
    if (emergencyStopCallback_) emergencyStopCallback_();
}
