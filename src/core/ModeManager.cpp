// src/core/ModeManager.cpp

#include "core/ModeManager.h"
#include <Arduino.h>
#include <esp_task_wdt.h>
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
    esp_task_wdt_init(5, true);
    esp_task_wdt_add(NULL);
    
    if (cfg_.estop_pin >= 0) pinMode(cfg_.estop_pin, INPUT_PULLUP);
    if (cfg_.bypass_pin >= 0) pinMode(cfg_.bypass_pin, INPUT_PULLUP);
    if (cfg_.relay_pin >= 0) {
        pinMode(cfg_.relay_pin, OUTPUT);
        digitalWrite(cfg_.relay_pin, LOW);
    }
    
    mode_ = RobotMode::DISCONNECTED;
}

void ModeManager::update(uint32_t now_ms) {
    esp_task_wdt_reset();
    readHardwareInputs();
    
    // Host timeout
    if (hostEverSeen_ && isConnected() && !isEstopped()) {
        uint32_t dt = now_ms - lastHostHeartbeat_;
    if (dt > cfg_.host_timeout_ms) {
        triggerStop();

        if (mode_ == RobotMode::ACTIVE) {
            mode_ = RobotMode::ARMED;     // fall back from ACTIVE
        } else if (mode_ == RobotMode::ARMED) {
            mode_ = RobotMode::ARMED;     // stay ARMED (don’t disarm)
        } else {
            mode_ = RobotMode::IDLE;
        }

        lastHostHeartbeat_ = now_ms;      // prevent retrigger spam
    }

    }
    
    // Motion timeout (only in ACTIVE)
    if (mode_ == RobotMode::ACTIVE && lastMotionCmd_ > 0) {
        uint32_t dtm = now_ms - lastMotionCmd_;
        if (dtm > cfg_.motion_timeout_ms) {
            triggerStop();
            mode_ = RobotMode::ARMED;

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

    // Relay control
    if (cfg_.relay_pin >= 0) {
        bool allow = canMove() && !isEstopped();
        digitalWrite(cfg_.relay_pin, allow ? HIGH : LOW);
    }
}

void ModeManager::readHardwareInputs() {
    if (cfg_.bypass_pin >= 0) {
        bypassed_ = (digitalRead(cfg_.bypass_pin) == LOW);
    }
    if (cfg_.estop_pin >= 0) {
        if (digitalRead(cfg_.estop_pin) == LOW && !isEstopped()) {
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

void ModeManager::onMotionCommand(uint32_t now_ms) {
    lastMotionCmd_ = now_ms;
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
    if (mode_ == RobotMode::IDLE) {
        mode_ = RobotMode::ARMED;
    }
}

void ModeManager::activate() {
    if (mode_ == RobotMode::ARMED) {
        lastMotionCmd_ = millis();
        mode_ = RobotMode::ACTIVE;
    }
}

void ModeManager::deactivate() {
    if (mode_ == RobotMode::ACTIVE) {
        triggerStop();
        mode_ = RobotMode::ARMED;
        lastMotionCmd_ = millis();
    }
}

void ModeManager::disarm() {
    if (mode_ == RobotMode::ARMED || mode_ == RobotMode::ACTIVE) {
        triggerStop();
        mode_ = RobotMode::IDLE;
    }
}

void ModeManager::estop() {
    triggerStop();
    mode_ = RobotMode::ESTOPPED;
}

bool ModeManager::clearEstop() {
    if (!isEstopped()) return true;
    
    // Hardware E-stop must be released
    if (cfg_.estop_pin >= 0 && digitalRead(cfg_.estop_pin) == LOW) {
        return false;
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