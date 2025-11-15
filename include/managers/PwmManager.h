#pragma once
#include <Arduino.h>
#include <map>

class PwmManager {
public:
    void registerChannel(int ch, int pin, int ledcChannel, float defaultFreq = 50.0f) {
        channelToPin_[ch] = pin;
        channelToLedc_[ch] = ledcChannel;
        freqDefault_[ch]   = defaultFreq;

        ledcSetup(ledcChannel, defaultFreq, 12); // 12-bit resolution
        ledcAttachPin(pin, ledcChannel);
    }

    void set(int ch, float duty, float freq=0.0f) {
        if (!exists(ch)) return;

        int ledcCH = channelToLedc_[ch];

        if (freq > 0.0f) {
            ledcSetup(ledcCH, freq, 12);
        }

        uint32_t dutyVal = (uint32_t)(duty * 4095);
        ledcWrite(ledcCH, dutyVal);
    }

private:
    bool exists(int ch) {
        if (!channelToPin_.count(ch)) {
            Serial.printf("[PWM] Unknown channel %d\n", ch);
            return false;
        }
        return true;
    }

    std::map<int,int> channelToPin_;
    std::map<int,int> channelToLedc_;
    std::map<int,float> freqDefault_;
};
