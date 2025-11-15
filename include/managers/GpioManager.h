#pragma once
#include <Arduino.h>
#include <map>

class GpioManager {
public:
    void registerChannel(int ch, int pin) {
        pinMode(pin, OUTPUT);
        channelToPin_[ch] = pin;
    }

    void write(int ch, int value) {
        auto it = channelToPin_.find(ch);
        if (it == channelToPin_.end()) {
            Serial.printf("[GPIO] Unknown channel %d\n", ch);
            return;
        }
        digitalWrite(it->second, value ? HIGH : LOW);
    }

private:
    std::map<int,int> channelToPin_;
};
