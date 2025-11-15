#pragma once
#include <Arduino.h>
#include <map>
#include <ESP32Servo.h>

class ServoManager {
public:
    void attach(int servoId, int pin, int minUs=1000, int maxUs=2000) {
        Servo s;
        s.attach(pin, minUs, maxUs);
        servos_[servoId] = s;
    }

    void detach(int servoId) {
        if (!servos_.count(servoId)) return;
        servos_[servoId].detach();
        servos_.erase(servoId);
    }

    void setAngle(int servoId, float angle) {
        if (!servos_.count(servoId)) {
            Serial.printf("[SERVO] Unknown servo id=%d\n", servoId);
            return;
        }
        servos_[servoId].write(angle);
    }

private:
    std::map<int, Servo> servos_;
};
