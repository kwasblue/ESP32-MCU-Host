#pragma once

#include <Arduino.h>

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
            Serial.printf("[UltrasonicManager] attach failed, id=%u out of range\n", id);
            return false;
        }

        pinMode(trigPin, OUTPUT);
        pinMode(echoPin, INPUT);

        sensors_[id].trigPin   = trigPin;
        sensors_[id].echoPin   = echoPin;
        sensors_[id].attached  = true;

        Serial.printf(
            "[UltrasonicManager] attach id=%u trig=%d echo=%d\n",
            id, trigPin, echoPin
        );
        return true;
    }

    bool isAttached(uint8_t id) const {
        return (id < MAX_SENSORS) && sensors_[id].attached;
    }

    // Returns distance in cm, or negative on error
    float readDistanceCm(uint8_t id) {
        if (id >= MAX_SENSORS || !sensors_[id].attached) {
            Serial.printf("[UltrasonicManager] read ignored, id=%u not attached\n", id);
            return -1.0f;
        }

        auto& s = sensors_[id];

        // Trigger pulse
        digitalWrite(s.trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(s.trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(s.trigPin, LOW);

        // Measure echo
        unsigned long duration = pulseIn(s.echoPin, HIGH, 30000); // 30ms timeout

        if (duration == 0) {
            // timeout
            return -1.0f;
        }

        float distanceCm = (duration * 0.0343f) / 2.0f;
        return distanceCm;
    }

private:
    Sensor sensors_[MAX_SENSORS];
};
