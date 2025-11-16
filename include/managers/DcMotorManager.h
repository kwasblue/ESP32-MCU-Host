#pragma once

#include <Arduino.h>

class DcMotorManager {
public:
    static constexpr uint8_t MAX_MOTORS = 4;

    struct Motor {
        int in1Pin    = -1;
        int in2Pin    = -1;
        int pwmPin    = -1;
        int pwmChan   = -1;
        bool attached = false;
        float lastSpeed = 0.0f; // -1.0 .. +1.0
    };

    DcMotorManager() = default;

    bool attach(uint8_t id, int in1Pin, int in2Pin, int pwmPin,
                int pwmChannel, int freq = 20000, int resolutionBits = 8) {
        if (id >= MAX_MOTORS) {
            Serial.printf("[DcMotorManager] attach failed, id=%u out of range\n", id);
            return false;
        }

        pinMode(in1Pin, OUTPUT);
        pinMode(in2Pin, OUTPUT);
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, LOW);

        ledcSetup(pwmChannel, freq, resolutionBits);
        ledcAttachPin(pwmPin, pwmChannel);
        ledcWrite(pwmChannel, 0);

        motors_[id].in1Pin    = in1Pin;
        motors_[id].in2Pin    = in2Pin;
        motors_[id].pwmPin    = pwmPin;
        motors_[id].pwmChan   = pwmChannel;
        motors_[id].attached  = true;
        motors_[id].lastSpeed = 0.0f;

        Serial.printf(
            "[DcMotorManager] attach id=%u in1=%d in2=%d pwm=%d ch=%d\n",
            id, in1Pin, in2Pin, pwmPin, pwmChannel
        );
        return true;
    }

    bool isAttached(uint8_t id) const {
        return (id < MAX_MOTORS) && motors_[id].attached;
    }

    // speed: -1.0 (full reverse) .. 0 .. +1.0 (full forward)
    bool setSpeed(uint8_t id, float speed) {
        if (id >= MAX_MOTORS || !motors_[id].attached) {
            Serial.printf("[DcMotorManager] setSpeed ignored, id=%u not attached\n", id);
            return false;
        }

        // Clamp
        if (speed > 1.0f) speed = 1.0f;
        if (speed < -1.0f) speed = -1.0f;

        auto& m = motors_[id];
        m.lastSpeed = speed;

        int duty = (int)(fabs(speed) * 255.0f); // 8-bit resolution

        if (speed > 0.0f) {
            digitalWrite(m.in1Pin, HIGH);
            digitalWrite(m.in2Pin, LOW);
        } else if (speed < 0.0f) {
            digitalWrite(m.in1Pin, LOW);
            digitalWrite(m.in2Pin, HIGH);
        } else {
            // Brake or coast – here we choose brake:
            digitalWrite(m.in1Pin, LOW);
            digitalWrite(m.in2Pin, LOW);
        }

        ledcWrite(m.pwmChan, duty);
        Serial.printf("[DcMotorManager] id=%u speed=%.2f duty=%d\n", id, speed, duty);
        return true;
    }

    bool stop(uint8_t id) {
        return setSpeed(id, 0.0f);
    }

private:
    Motor motors_[MAX_MOTORS];
};
