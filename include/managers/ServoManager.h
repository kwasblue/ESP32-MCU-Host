#pragma once
#include <Arduino.h>
#include <ESP32Servo.h>

class ServoManager {
public:
    ServoManager() = default;

    void attach(int servoId, int pin, int minUs = 1000, int maxUs = 2000) {
        if (servoId != 0) {
            Serial.printf("[ServoManager] attach: unsupported servoId=%d (only 0 allowed)\n",
                          servoId);
            return;
        }

        Serial.printf("[ServoManager] attach id=%d pin=%d min=%d max=%d\n",
                      servoId, pin, minUs, maxUs);

        pin_   = pin;
        minUs_ = minUs;
        maxUs_ = maxUs;

        // attach() returns a channel number (0..15) or maybe -1 on failure,
        // so 0 is NOT a failure.
        int ch = servo_.attach(pin, minUs, maxUs);
        Serial.printf("[ServoManager] attach returned channel=%d\n", ch);

        // Be generous: consider anything >= 0 as success.
        attached_ = (ch >= 0);
    }

    void detach(int servoId) {
        if (servoId != 0) return;
        if (!attached_)  return;

        Serial.println("[ServoManager] detach id=0");
        servo_.detach();
        attached_ = false;
        pin_      = -1;
    }

    void setAngle(int servoId, float angleDeg) {
        if (servoId != 0) {
            Serial.printf("[ServoManager] setAngle: unsupported servoId=%d\n", servoId);
            return;
        }
        if (!attached_) {
            Serial.println("[ServoManager] setAngle ignored, not attached");
            return;
        }

        if (angleDeg < 0.0f)   angleDeg = 0.0f;
        if (angleDeg > 180.0f) angleDeg = 180.0f;

        Serial.printf("[ServoManager] setAngle id=0 angle=%.1f on pin=%d\n",
                      angleDeg, pin_);

        servo_.write(static_cast<int>(angleDeg));
    }

private:
    Servo servo_;          // exactly like your working sketch
    bool  attached_ = false;
    int   pin_      = -1;
    int   minUs_    = 1000;
    int   maxUs_    = 2000;
};
