// include/core/MotionController.h
#pragma once

#include <Arduino.h>
#include "managers/DcMotorManager.h"
#include "managers/ServoManager.h"

// MotionController:
// - Drives a differential-drive base using DcMotorManager (vx, omega in SI units)
// - Smooths vx/omega over time (accel limits)
// - Optionally interpolates servo angles over time using ServoManager
class MotionController {
public:
    static constexpr uint8_t ESP_MAX_SERVOS = 1;  // bump when you add more

    MotionController(DcMotorManager& motors,
                     uint8_t leftMotorId,
                     uint8_t rightMotorId,
                     float wheelBase,
                     float maxLinear,
                     float maxAngular,
                     ServoManager* servoMgr = nullptr);

    // ===== Differential drive velocity interface =====
    void setVelocity(float vx, float omega);
    void stop();

    float vx() const;
    float omega() const;

    void setAccelLimits(float maxLinAccel, float maxAngAccel);

    // ===== Servo interface =====
    // durationMs == 0 => immediate jump (no interpolation)
    // durationMs  > 0 => interpolated motion handled in update()
    void setServoTarget(uint8_t servoId, float angleDeg, uint32_t durationMs);
    void setServoImmediate(uint8_t servoId, float angleDeg);

    // ===== Main update loop =====
    // Called once per loop with dt in seconds.
    void update(float dt);

private:
    // === DC motor control ===
    DcMotorManager& motors_;
    ServoManager*   servoMgr_;   // may be nullptr if no servos

    uint8_t leftId_;
    uint8_t rightId_;

    float wheelBase_;   // meters
    float maxLinear_;   // m/s (body)
    float maxAngular_;  // rad/s (body)

    // Reference commands from host
    float vxRef_    = 0.0f;
    float omegaRef_ = 0.0f;

    // What we're currently commanding after ramping
    float vxCmd_    = 0.0f;
    float omegaCmd_ = 0.0f;

    // Ramp/acceleration limits
    float maxLinAccel_ = 1.0f;  // m/s^2
    float maxAngAccel_ = 2.0f;  // rad/s^2

    // === Servo motion state ===
    float    servoCurrent_[MAX_SERVOS];    // last commanded angle
    float    servoStart_[MAX_SERVOS];      // start of trajectory
    float    servoTarget_[MAX_SERVOS];     // target angle
    uint32_t servoStartMs_[MAX_SERVOS];    // start time [ms]
    uint32_t servoDurationMs_[MAX_SERVOS]; // total move duration [ms]
    bool     servoActive_[MAX_SERVOS];     // trajectory active?
};
