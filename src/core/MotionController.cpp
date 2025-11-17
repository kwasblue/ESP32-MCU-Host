// src/core/MotionController.cpp
#include "core/MotionController.h"

// If you use Debug macros, you can include Debug.h, otherwise ignore
#include "core/Debug.h"

MotionController::MotionController(DcMotorManager& motors,
                                   uint8_t leftMotorId,
                                   uint8_t rightMotorId,
                                   float wheelBase,
                                   float maxLinear,
                                   float maxAngular,
                                   ServoManager* servoMgr)
    : motors_(motors),
      servoMgr_(servoMgr),
      leftId_(leftMotorId),
      rightId_(rightMotorId),
      wheelBase_(wheelBase),
      maxLinear_(maxLinear),
      maxAngular_(maxAngular) {
    // init servo state
    for (uint8_t i = 0; i < MAX_SERVOS; ++i) {
        servoCurrent_[i]    = 0.0f;
        servoStart_[i]      = 0.0f;
        servoTarget_[i]     = 0.0f;
        servoStartMs_[i]    = 0;
        servoDurationMs_[i] = 0;
        servoActive_[i]     = false;
    }
}

// ===== Differential drive =====

void MotionController::setVelocity(float vx, float omega) {
    if (vx >  maxLinear_) vx =  maxLinear_;
    if (vx < -maxLinear_) vx = -maxLinear_;
    if (omega >  maxAngular_) omega =  maxAngular_;
    if (omega < -maxAngular_) omega = -maxAngular_;

    vxRef_    = vx;
    omegaRef_ = omega;
}

void MotionController::stop() {
    vxRef_    = 0.0f;
    omegaRef_ = 0.0f;
    // Optionally: also zero motor outputs here via motors_
}

float MotionController::vx() const {
    return vxRef_;
}

float MotionController::omega() const {
    return omegaRef_;
}

void MotionController::setAccelLimits(float maxLinAccel, float maxAngAccel) {
    maxLinAccel_ = maxLinAccel;
    maxAngAccel_ = maxAngAccel;
}

// ===== Servo interface =====

void MotionController::setServoTarget(uint8_t servoId,
                                      float angleDeg,
                                      uint32_t durationMs) {
    if (!servoMgr_) return;
    if (servoId >= MAX_SERVOS) return;

    if (durationMs == 0) {
        // Immediate move: no trajectory, just write the angle now.
        servoCurrent_[servoId] = angleDeg;
        servoActive_[servoId]  = false;
        DBG_PRINTF("[MotionController] immediate servo id=%u angle=%.1f\n",
                   servoId, angleDeg);
        servoMgr_->setAngle(servoId, angleDeg);
        return;
    }

    // Schedule an interpolated move
    servoActive_[servoId]     = true;
    servoStart_[servoId]      = servoCurrent_[servoId];
    servoTarget_[servoId]     = angleDeg;
    servoStartMs_[servoId]    = millis();
    servoDurationMs_[servoId] = durationMs;

    DBG_PRINTF("[MotionController] servo id=%u target=%.1f dur=%lu ms (start=%.1f)\n",
               servoId, angleDeg, (unsigned long)durationMs, servoStart_[servoId]);
}

void MotionController::setServoImmediate(uint8_t servoId, float angleDeg) {
    setServoTarget(servoId, angleDeg, 0);
}

// ===== Main update =====

void MotionController::update(float dt) {
    // --- 1) Velocity ramping & motor output (placeholder) ---
    // You can fill this in later with:
    //   - ramp vxCmd_/omegaCmd_ toward vxRef_/omegaRef_
    //   - convert to left/right wheel speeds
    //   - motors_.setSpeed(leftId_, speedL), motors_.setSpeed(rightId_, speedR)

    (void)dt; // to silence unused parameter warning for now

    // --- 2) Servo interpolation ---
    if (!servoMgr_) return;

    uint32_t now = millis();
    for (uint8_t id = 0; id < MAX_SERVOS; ++id) {
        if (!servoActive_[id]) continue;

        uint32_t elapsed = now - servoStartMs_[id];
        if (elapsed >= servoDurationMs_[id]) {
            // Final position
            servoCurrent_[id] = servoTarget_[id];
            servoMgr_->setAngle(id, servoCurrent_[id]);
            servoActive_[id] = false;
            DBG_PRINTF("[MotionController] servo id=%u done at %.1f\n",
                       id, servoCurrent_[id]);
        } else {
            float t = (float)elapsed / (float)servoDurationMs_[id];  // 0..1
            float angle = servoStart_[id] +
                          t * (servoTarget_[id] - servoStart_[id]);
            servoCurrent_[id] = angle;
            servoMgr_->setAngle(id, servoCurrent_[id]);
        }
    }
}

