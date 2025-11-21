#pragma once

#include <Arduino.h>
#include "core/Debug.h"
#include "managers/GpioManager.h"
#include "managers/PwmManager.h"

class DcMotorManager {
public:
    static constexpr uint8_t MAX_MOTORS = 4;

    struct Motor {
        // Physical pins (for debug / introspection)
        int in1Pin    = -1;
        int in2Pin    = -1;
        int pwmPin    = -1;

        // Hardware LEDC channel (0..15)
        int ledcChannel = -1;

        // Logical channels in managers
        int gpioChIn1  = -1;
        int gpioChIn2  = -1;
        int pwmCh      = -1;

        bool  attached   = false;
        float lastSpeed  = 0.0f;  // -1.0 .. +1.0
        float freqHz     = 0.0f;
        int   resolution = 0;
    };

    // Public debug view
    struct MotorDebugInfo {
        uint8_t id         = 0;
        bool    attached   = false;

        int in1Pin         = -1;
        int in2Pin         = -1;
        int pwmPin         = -1;
        int ledcChannel    = -1;

        int gpioChIn1      = -1;
        int gpioChIn2      = -1;
        int pwmCh          = -1;

        float lastSpeed    = 0.0f;
        float freqHz       = 0.0f;
        int   resolution   = 0;
    };

    DcMotorManager(GpioManager& gpio, PwmManager& pwm)
        : gpio_(gpio), pwm_(pwm) {}

    // Attach a DC motor to direction + PWM pins
    bool attach(uint8_t id,
                int in1Pin,
                int in2Pin,
                int pwmPin,
                int ledcChannel,
                int freq = 20000,
                int resolutionBits = 12)
    {
        if (id >= MAX_MOTORS) {
            DBG_PRINTF("[DcMotorManager] attach failed, id=%u out of range\n", id);
            return false;
        }

        // Derive logical channels from motor id.
        const int gpioChIn1 = id * 2;
        const int gpioChIn2 = id * 2 + 1;
        const int pwmCh     = id;

        // Configure direction pins via GpioManager
        gpio_.registerChannel(gpioChIn1, in1Pin, OUTPUT);
        gpio_.registerChannel(gpioChIn2, in2Pin, OUTPUT);
        gpio_.write(gpioChIn1, LOW);
        gpio_.write(gpioChIn2, LOW);

        // Configure PWM via PwmManager
        pwm_.registerChannel(pwmCh, pwmPin, ledcChannel, static_cast<float>(freq));
        pwm_.set(pwmCh, 0.0f);

        Motor& m      = motors_[id];
        m.in1Pin      = in1Pin;
        m.in2Pin      = in2Pin;
        m.pwmPin      = pwmPin;
        m.ledcChannel = ledcChannel;
        m.gpioChIn1   = gpioChIn1;
        m.gpioChIn2   = gpioChIn2;
        m.pwmCh       = pwmCh;
        m.attached    = true;
        m.lastSpeed   = 0.0f;
        m.freqHz      = static_cast<float>(freq);
        m.resolution  = resolutionBits;

        DBG_PRINTF(
            "[DcMotorManager] attach id=%u in1=%d in2=%d pwmPin=%d ledcCH=%d "
            "gpioChIn1=%d gpioChIn2=%d pwmCh=%d freq=%d res=%d\n",
            id, in1Pin, in2Pin, pwmPin, ledcChannel,
            gpioChIn1, gpioChIn2, pwmCh, freq, resolutionBits
        );
        return true;
    }

    bool isAttached(uint8_t id) const {
        return (id < MAX_MOTORS) && motors_[id].attached;
    }

    // speed: -1.0 (full reverse) .. 0 .. +1.0 (full forward)
    bool setSpeed(uint8_t id, float speed) {
        if (id >= MAX_MOTORS || !motors_[id].attached) {
            DBG_PRINTF("[DcMotorManager] setSpeed ignored, id=%u not attached\n", id);
            return false;
        }

        // Clamp speed to [-1, 1]
        if (speed >  1.0f) speed =  1.0f;
        if (speed < -1.0f) speed = -1.0f;

        Motor& m = motors_[id];
        m.lastSpeed = speed;

        const float mag = fabsf(speed);  // 0..1, for PWM duty

        // Direction via GpioManager
        if (mag == 0.0f) {
            gpio_.write(m.gpioChIn1, LOW);
            gpio_.write(m.gpioChIn2, LOW);
        } else if (speed > 0.0f) {
            gpio_.write(m.gpioChIn1, HIGH);
            gpio_.write(m.gpioChIn2, LOW);
        } else {
            gpio_.write(m.gpioChIn1, LOW);
            gpio_.write(m.gpioChIn2, HIGH);
        }

        // PWM duty via PwmManager (0..1)
        pwm_.set(m.pwmCh, mag);

        DBG_PRINTF("[DcMotorManager] id=%u speed=%.3f mag=%.3f\n",
                   id, speed, mag);
        return true;
    }

    bool stop(uint8_t id) {
        return setSpeed(id, 0.0f);
    }

    void stopAll() {
        for (uint8_t i = 0; i < MAX_MOTORS; ++i) {
            if (motors_[i].attached) {
                setSpeed(i, 0.0f);
            }
        }
    }

    // === Debug helpers ===
    bool getMotorDebugInfo(uint8_t id, MotorDebugInfo& out) const {
        if (id >= MAX_MOTORS) {
            return false;
        }
        const Motor& m = motors_[id];

        out.id         = id;
        out.attached   = m.attached;
        out.in1Pin     = m.in1Pin;
        out.in2Pin     = m.in2Pin;
        out.pwmPin     = m.pwmPin;
        out.ledcChannel= m.ledcChannel;
        out.gpioChIn1  = m.gpioChIn1;
        out.gpioChIn2  = m.gpioChIn2;
        out.pwmCh      = m.pwmCh;
        out.lastSpeed  = m.lastSpeed;
        out.freqHz     = m.freqHz;
        out.resolution = m.resolution;

        return m.attached;
    }

    void dumpAllMotorMappings() const {
        DBG_PRINTF("=== DcMotorManager mappings ===\n");
        for (uint8_t id = 0; id < MAX_MOTORS; ++id) {
            const Motor& m = motors_[id];
            if (!m.attached) {
                DBG_PRINTF("  id=%u: [NOT ATTACHED]\n", id);
                continue;
            }

            DBG_PRINTF(
                "  id=%u: in1Pin=%d in2Pin=%d pwmPin=%d ledcCH=%d "
                "gpioChIn1=%d gpioChIn2=%d pwmCh=%d freq=%.1fHz "
                "res=%d lastSpeed=%.3f\n",
                id,
                m.in1Pin,
                m.in2Pin,
                m.pwmPin,
                m.ledcChannel,
                m.gpioChIn1,
                m.gpioChIn2,
                m.pwmCh,
                m.freqHz,
                m.resolution,
                m.lastSpeed
            );
        }
        DBG_PRINTF("=== end DcMotorManager mappings ===\n");
    }

private:
    GpioManager& gpio_;
    PwmManager&  pwm_;

    Motor motors_[MAX_MOTORS];
};
