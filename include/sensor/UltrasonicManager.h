#pragma once

#include "config/FeatureFlags.h"

#if HAS_ULTRASONIC

#include <Arduino.h>
#include "core/Debug.h"

class UltrasonicManager {
public:
    static constexpr uint8_t MAX_SENSORS = 4;

    struct Sensor {
        int trigPin   = -1;
        int echoPin   = -1;
        bool attached = false;
    };

    UltrasonicManager() = default;

    bool attach(uint8_t id, int trigPin, int echoPin) {
        if (id >= MAX_SENSORS) {
            DBG_PRINTF("[UltrasonicManager] attach failed, id=%u out of range\n", id);
            return false;
        }

        pinMode(trigPin, OUTPUT);
        pinMode(echoPin, INPUT);

        sensors_[id].trigPin  = trigPin;
        sensors_[id].echoPin  = echoPin;
        sensors_[id].attached = true;

        DBG_PRINTF("[UltrasonicManager] attach id=%u trig=%d echo=%d\n",
                   id, trigPin, echoPin);

        return true;
    }

    bool isAttached(uint8_t id) const {
        return (id < MAX_SENSORS) && sensors_[id].attached;
    }

    // Returns distance (cm), or negative on error/timeout
    float readDistanceCm(uint8_t id) {
        if (id >= MAX_SENSORS || !sensors_[id].attached) {
            DBG_PRINTF("[UltrasonicManager] read ignored, id=%u not attached\n", id);
            return -1.0f;
        }

        auto& s = sensors_[id];

        // Trigger pulse
        digitalWrite(s.trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(s.trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(s.trigPin, LOW);

        // Measure echo (timeout ~30ms)
        unsigned long duration = pulseIn(s.echoPin, HIGH, 30000);

        if (duration == 0) {
            DBG_PRINTF("[UltrasonicManager] id=%u timeout/no echo\n", id);
            return -1.0f;
        }

        // Speed of sound ≈ 0.0343 cm/µs; divide by 2 (round trip)
        float distance = (duration * 0.0343f) / 2.0f;

        DBG_PRINTF("[UltrasonicManager] id=%u distance=%.2f cm\n", id, distance);

        return distance;
    }

private:
    Sensor sensors_[MAX_SENSORS];
};

#else // !HAS_ULTRASONIC

// Stub when ultrasonic is disabled
class UltrasonicManager {
public:
    static constexpr uint8_t MAX_SENSORS = 4;

    struct Sensor {
        int trigPin = -1;
        int echoPin = -1;
        bool attached = false;
    };

    UltrasonicManager() = default;
    bool attach(uint8_t, int, int) { return false; }
    bool isAttached(uint8_t) const { return false; }
    float readDistanceCm(uint8_t) { return -1.0f; }
};

#endif // HAS_ULTRASONIC
