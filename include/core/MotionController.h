// include/core/MotionController.h
#pragma once

#include <Arduino.h>

class DcMotorManager;
class ServoManager;
class StepperManager;

// MotionController:
// - Drives a differential-drive base using DcMotorManager (vx, omega in SI units)
// - Smooths vx/omega over time (accel limits)
// - Optionally interpolates servo angles over time using ServoManager
// - Optionally commands steppers via StepperManager
class MotionController {
public:
    static constexpr uint8_t ESP_MAX_SERVOS = 1;  // bump when you add more

    MotionController(DcMotorManager& motors,
                     uint8_t leftMotorId,
                     uint8_t rightMotorId,
                     float wheelBase,
                     float maxLinear,
                     float maxAngular,
                     ServoManager*  servoMgr = nullptr,
                     StepperManager* stepperMgr = nullptr);

    // ===== Differential drive velocity interface =====
    void setVelocity(float vx, float omega);
    void stop();

    float vx() const;
    float omega() const;

    void setAccelLimits(float maxLinAccel, float maxAngAccel);

    // Enable/disable base control (NEW)
    void setBaseEnabled(bool enabled);
    bool baseEnabled() const { return baseEnabled_; }

    // ===== Servo interface =====
    void setServoTarget(uint8_t servoId,
                        float angleDeg,
                        uint32_t durationMs = 0);
    void setServoImmediate(uint8_t servoId, float angleDeg);

    // ===== Stepper interface =====
    void moveStepperRelative(int motorId,
                             int steps,
                             float speedStepsPerSec);
    void enableStepper(int motorId, bool enabled);

    // ===== Main update (called every loop) =====
    void update(float dt);

private:
    // References
    DcMotorManager& motors_;
    ServoManager*   servoMgr_    = nullptr;
    StepperManager* stepperMgr_  = nullptr;

    // Base config
    uint8_t leftId_   = 0;
    uint8_t rightId_  = 1;
    float   wheelBase_ = 0.2f;
    float   maxLinear_ = 0.5f;
    float   maxAngular_ = 1.0f;

    // Velocity state
    float vxRef_      = 0.0f;
    float omegaRef_   = 0.0f;
    float vxCmd_      = 0.0f;
    float omegaCmd_   = 0.0f;
    float maxLinAccel_  = 1.0f;
    float maxAngAccel_  = 2.0f;

    // Whether MotionController is allowed to drive the base (NEW)
    bool baseEnabled_ = false;

    // Servo trajectory state
    float    servoCurrent_[ESP_MAX_SERVOS];
    float    servoStart_[ESP_MAX_SERVOS];
    float    servoTarget_[ESP_MAX_SERVOS];
    uint32_t servoStartMs_[ESP_MAX_SERVOS];
    uint32_t servoDurationMs_[ESP_MAX_SERVOS];
    bool     servoActive_[ESP_MAX_SERVOS];
};
