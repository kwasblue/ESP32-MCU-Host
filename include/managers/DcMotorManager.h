#pragma once

#include <Arduino.h>
#include <math.h>          // for fabsf
#include "core/Debug.h"
#include "managers/GpioManager.h"
#include "managers/PwmManager.h"
#include "core/PID.h"      // 🔹 NEW: your PID class

class DcMotorManager {
public:
    static constexpr uint8_t MAX_MOTORS = 4;

    // Match auto-generated GPIO_CHANNEL_DEFS:
    // 0: LED_STATUS
    // 1: ULTRASONIC_TRIG
    // 2: ULTRASONIC_ECHO
    // 3: MOTOR_LEFT_IN1
    // 4: MOTOR_LEFT_IN2
    // 5: STEPPER0_EN
    // 6: ENC0_A
    // 7: ENC0_B
    //
    // So DC motors will start at channel 3:
    //   motor 0: in1 -> 3, in2 -> 4
    //   motor 1: in1 -> 5, in2 -> 6 (if you ever want it)
    static constexpr int GPIO_BASE_CH = 3;   // base for IN1/IN2 channels
    static constexpr int PWM_BASE_CH  = 0;   // logical PWM channels 0,1,2,...

    struct Motor {
        int in1Pin      = -1;
        int in2Pin      = -1;
        int pwmPin      = -1;

        int ledcChannel = -1;  // hardware LEDC channel (0..15)

        int gpioChIn1   = -1;  // logical channels in GpioManager
        int gpioChIn2   = -1;
        int pwmCh       = -1;  // logical PWM channel in PwmManager

        bool  attached   = false;
        float lastSpeed  = 0.0f;  // -1.0 .. +1.0
        float freqHz     = 0.0f;
        int   resolution = 0;

        // === PID / velocity control additions ===
        bool  pidEnabled      = false;   // closed-loop velocity control enabled
        float targetOmegaRadS = 0.0f;    // desired velocity [rad/s]
        PID   pid;                       // PID controller (output -> duty -1..1)
    };

    struct MotorDebugInfo {
        uint8_t id       = 0;
        bool    attached = false;

        int in1Pin       = -1;
        int in2Pin       = -1;
        int pwmPin       = -1;
        int ledcChannel  = -1;

        int gpioChIn1    = -1;
        int gpioChIn2    = -1;
        int pwmCh        = -1;

        float lastSpeed  = 0.0f;
        float freqHz     = 0.0f;
        int   resolution = 0;

        // PID debug
        bool  pidEnabled      = false;
        float targetOmegaRadS = 0.0f;
    };

    DcMotorManager(GpioManager& gpio, PwmManager& pwm)
        : gpio_(gpio), pwm_(pwm)
    {
        // Initialize PID defaults for all motors
        for (uint8_t i = 0; i < MAX_MOTORS; ++i) {
            motors_[i].pid.setOutputLimits(-1.0f, 1.0f);  // duty range
            motors_[i].pid.reset();
            motors_[i].pidEnabled      = false;
            motors_[i].targetOmegaRadS = 0.0f;
        }
    }

    bool attach(uint8_t id,
                int in1Pin,
                int in2Pin,
                int pwmPin,
                int ledcChannel,
                int freq = 15000,          // 🔹 lowered from 20000
                int resolutionBits = 12)
    {
        if (id >= MAX_MOTORS) {
            DBG_PRINTF("[DcMotorManager] attach failed, id=%u out of range\n", id);
            return false;
        }

        // Use a base that matches the auto-generated channels.
        const int gpioChIn1 = GPIO_BASE_CH + id * 2;
        const int gpioChIn2 = GPIO_BASE_CH + id * 2 + 1;
        const int pwmCh     = PWM_BASE_CH  + id;

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

        // PID defaults for this motor
        m.pid.setOutputLimits(-1.0f, 1.0f);
        m.pid.reset();
        m.pidEnabled      = false;
        m.targetOmegaRadS = 0.0f;

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

    // speed: -1.0 .. +1.0
    // NOTE: This is still your low-level "open-loop" command.
    // When PID is enabled, we call this from updateVelocityPid()
    // with the controller output.
    bool setSpeed(uint8_t id, float speed) {
        if (id >= MAX_MOTORS || !motors_[id].attached) {
            DBG_PRINTF("[DcMotorManager] setSpeed ignored, id=%u not attached\n", id);
            return false;
        }

        if (speed >  1.0f) speed =  1.0f;
        if (speed < -1.0f) speed = -1.0f;

        Motor& m     = motors_[id];
        m.lastSpeed  = speed;
        const float mag = fabsf(speed);  // 0..1

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

        // PWM duty via PwmManager
        pwm_.set(m.pwmCh, mag);

        DBG_PRINTF("[DcMotorManager] id=%u speed=%.3f mag=%.3f\n",
                   id, speed, mag);
        return true;
    }

    // Stop motor: also reset PID target -> 0 if enabled
    bool stop(uint8_t id) {
        if (id >= MAX_MOTORS || !motors_[id].attached) {
            return false;
        }
        Motor& m = motors_[id];
        m.pidEnabled = false;
        m.targetOmegaRadS = 0.0f;
        if (m.pidEnabled) {
            m.pid.reset();
        }
        return setSpeed(id, 0.0f);
    }

    void stopAll() {
        for (uint8_t i = 0; i < MAX_MOTORS; ++i) {
            if (motors_[i].attached) {
                stop(i);
            }
        }
    }

    bool getMotorDebugInfo(uint8_t id, MotorDebugInfo& out) const {
        if (id >= MAX_MOTORS) {
            return false;
        }
        const Motor& m = motors_[id];

        out.id          = id;
        out.attached    = m.attached;
        out.in1Pin      = m.in1Pin;
        out.in2Pin      = m.in2Pin;
        out.pwmPin      = m.pwmPin;
        out.ledcChannel = m.ledcChannel;
        out.gpioChIn1   = m.gpioChIn1;
        out.gpioChIn2   = m.gpioChIn2;
        out.pwmCh       = m.pwmCh;
        out.lastSpeed   = m.lastSpeed;
        out.freqHz      = m.freqHz;
        out.resolution  = m.resolution;

        out.pidEnabled      = m.pidEnabled;
        out.targetOmegaRadS = m.targetOmegaRadS;

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
                "res=%d lastSpeed=%.3f pidEnabled=%d targetOmega=%.3f rad/s\n",
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
                m.lastSpeed,
                m.pidEnabled ? 1 : 0,
                m.targetOmegaRadS
            );
        }
        DBG_PRINTF("=== end DcMotorManager mappings ===\n");
    }

    // =========================
    // PID / velocity control API
    // =========================

    // Enable or disable closed-loop velocity control for this motor.
    // When enabled, you should call updateVelocityPid() periodically
    // with measuredOmega and dt.
    bool enableVelocityPid(uint8_t id, bool enable) {
        if (id >= MAX_MOTORS || !motors_[id].attached) {
            DBG_PRINTF("[DcMotorManager] enableVelocityPid ignored, id=%u not attached\n", id);
            return false;
        }
        Motor& m = motors_[id];
        m.pidEnabled = enable;
        m.pid.reset();
        DBG_PRINTF("[DcMotorManager] id=%u PID %s\n", id, enable ? "ENABLED" : "DISABLED");
        return true;
    }

    bool isVelocityPidEnabled(uint8_t id) const {
        if (id >= MAX_MOTORS || !motors_[id].attached) {
            return false;
        }
        return motors_[id].pidEnabled;
    }

    // Set desired velocity in rad/s for the motor.
    bool setVelocityTarget(uint8_t id, float omegaRadPerSec) {
        if (id >= MAX_MOTORS || !motors_[id].attached) {
            DBG_PRINTF("[DcMotorManager] setVelocityTarget ignored, id=%u not attached\n", id);
            return false;
        }
        motors_[id].targetOmegaRadS = omegaRadPerSec;
        DBG_PRINTF("[DcMotorManager] id=%u targetOmega=%.3f rad/s\n", id, omegaRadPerSec);
        return true;
    }

    // Configure PID gains for this motor.
    bool setVelocityGains(uint8_t id, float kp, float ki, float kd) {
        if (id >= MAX_MOTORS || !motors_[id].attached) {
            DBG_PRINTF("[DcMotorManager] setVelocityGains ignored, id=%u not attached\n", id);
            return false;
        }
        Motor& m = motors_[id];
        m.pid.setGains(kp, ki, kd);
        DBG_PRINTF("[DcMotorManager] id=%u PID gains kp=%.4f ki=%.4f kd=%.4f\n",
                   id, kp, ki, kd);
        return true;
    }

    // Call this at a fixed frequency (e.g., 500–1000 Hz) with the
    // measured angular velocity [rad/s] from your encoder and dt in seconds.
    // This computes a new duty command and applies it via setSpeed().
    bool updateVelocityPid(uint8_t id, float measuredOmegaRadS, float dt) {
        if (id >= MAX_MOTORS || !motors_[id].attached) {
            return false;
        }
        Motor& m = motors_[id];
        if (!m.pidEnabled) {
            return false;
        }
        if (dt <= 0.0f) {
            return false;
        }

        float cmd = m.pid.compute(m.targetOmegaRadS, measuredOmegaRadS, dt);
        // cmd should be within -1..1 due to PID output limits
        return setSpeed(id, cmd);
    }

private:
    GpioManager& gpio_;
    PwmManager&  pwm_;

    Motor motors_[MAX_MOTORS];
};
