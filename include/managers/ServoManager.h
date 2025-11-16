#pragma once
#include <Arduino.h>
#include <ESP32Servo.h>

class ServoManager {
public:
    ServoManager() = default;

    void attach(int servoId, int pin, int minUs = 500, int maxUs = 2400) {
        if (servoId != 0) {
            Serial.printf("[ServoManager] attach: unsupported servoId=%d (only 0 allowed)\n",
                          servoId);
            return;
        }

        pin_   = pin;
        minUs_ = minUs;
        maxUs_ = maxUs;

        Serial.printf("[ServoManager] attach id=%d pin=%d min=%d max=%d\n",
                      servoId, pin, minUs, maxUs);

        // IMPORTANT: no setPeriodHertz() here – it’s optional and was breaking your program.
        int ch = servo_.attach(pin, minUs, maxUs);
        Serial.printf("[ServoManager] attach returned channel=%d\n", ch);

        attached_ = (ch >= 0);
        if (!attached_) {
            Serial.println("[ServoManager] attach FAILED");
        }
    }

    void detach(int servoId) {
        if (servoId != 0) return;
        if (!attached_)  return;

        Serial.printf("[ServoManager] detach id=%d\n", servoId);
        servo_.detach();
        attached_ = false;
        pin_ = -1;
    }

    void setAngle(int servoId, float angleDeg) {
        if (servoId != 0) return;
        if (!attached_) {
            Serial.println("[ServoManager] setAngle ignored, not attached");
            return;
        }

        // Optional logical calibration
        float logical = angleDeg;

        // Apply offset/scale if you want:
        logical = offsetDeg_ + scale_ * logical;

        // Clamp to [0, 180] for the servo library
        if (logical < 0.0f)   logical = 0.0f;
        if (logical > 180.0f) logical = 180.0f;

        Serial.printf("[ServoManager] setAngle id=%d cmd=%.1f internal=%.1f on pin=%d\n",
                      servoId, angleDeg, logical, pin_);

        servo_.write(logical);
    }

    // If you want to tweak behavior at runtime later:
    void setOffset(float offsetDeg) { offsetDeg_ = offsetDeg; }
    void setScale(float scale)      { scale_ = scale; }

private:
    Servo servo_;
    bool  attached_ = false;
    int   pin_      = -1;
    int   minUs_    = 500;
    int   maxUs_    = 2400;

    // logicalAngle → internalAngle = offsetDeg_ + scale_ * logicalAngle
    float offsetDeg_ = 0.0f;
    float scale_     = 2.0f; // scaled 2x because for some reason the commands only go to half
};
