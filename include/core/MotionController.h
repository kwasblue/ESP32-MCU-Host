// include/core/MotionController.h
#pragma once

#include <Arduino.h>
#include "managers/DcMotorManager.h"
#include "managers/ServoManager.h"
#include "managers/StepperManager.h"

class MotionController {
public:
    static constexpr uint8_t ESP_MAX_SERVOS = 1;  // bump when you add more

    MotionController(DcMotorManager& motors,
                     uint8_t leftMotorId,
                     uint8_t rightMotorId,
                     float wheelBase,
                     float maxLinear,
                     float maxAngular,
                     ServoManager*  servoMgr    = nullptr,
                     StepperManager* stepperMgr = nullptr);

    // ===== Differential drive velocity interface =====
    void setVelocity(float vx, float omega);
    void stop();

    float vx() const;
    float omega() const;

    void setAccelLimits(float maxLinAccel, float maxAngAccel);

    // ===== Servo interface =====
    void setServoTarget(uint8_t servoId, float angleDeg, uint32_t durationMs);
    void setServoImmediate(uint8_t servoId, float angleDeg);

    // ===== Stepper interface =====
    void moveStepperRelative(int motorId,
                             int steps,
                             float speedStepsPerSec = 1000.0f);
    void enableStepper(int motorId, bool enabled);

    // ===== Main update loop =====
    void update(float dt);

private:
    // === DC motor control ===
    DcMotorManager& motors_;
    ServoManager*   servoMgr_;    // may be nullptr
    StepperManager* stepperMgr_;  // may be nullptr

    uint8_t leftId_;
    uint8_t rightId_;

    float wheelBase_;
    float maxLinear_;
    float maxAngular_;

    // Reference commands from host
    float vxRef_    = 0.0f;
    float omegaRef_ = 0.0f;

    // Ramped commands
    float vxCmd_    = 0.0f;
    float omegaCmd_ = 0.0f;

    float maxLinAccel_ = 1.0f;
    float maxAngAccel_ = 2.0f;

    // === Servo motion state ===
    float    servoCurrent_[ESP_MAX_SERVOS];
    float    servoStart_[ESP_MAX_SERVOS];
    float    servoTarget_[ESP_MAX_SERVOS];
    uint32_t servoStartMs_[ESP_MAX_SERVOS];
    uint32_t servoDurationMs_[ESP_MAX_SERVOS];
    bool     servoActive_[ESP_MAX_SERVOS];
};
