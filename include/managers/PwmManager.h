#pragma once
#include <Arduino.h>
#include <map>
#include "core/Debug.h"     // <-- add this

class PwmManager {
public:
    void registerChannel(int ch, int pin, int ledcChannel, float defaultFreq = 50.0f) {
        channelToPin_[ch] = pin;
        channelToLedc_[ch] = ledcChannel;
        freqDefault_[ch]   = defaultFreq;

        ledcSetup(ledcChannel, defaultFreq, 12); // 12-bit resolution
        ledcAttachPin(pin, ledcChannel);

        DBG_PRINTF("[PWM] registerChannel: ch=%d pin=%d ledcCH=%d freq=%.1f\n",
                   ch, pin, ledcChannel, defaultFreq);
    }

    void set(int ch, float duty, float freq=0.0f) {
        if (!exists(ch)) return;

        int ledcCH = channelToLedc_[ch];

        if (freq > 0.0f) {
            ledcSetup(ledcCH, freq, 12);
            DBG_PRINTF("[PWM] set: ch=%d freq override=%.1f\n", ch, freq);
        }

        uint32_t dutyVal = (uint32_t)(duty * 4095);
        ledcWrite(ledcCH, dutyVal);

        DBG_PRINTF("[PWM] set: ch=%d duty=%.3f dutyVal=%lu\n",
                   ch, duty, (unsigned long)dutyVal);
    }

private:
    bool exists(int ch) {
        if (!channelToPin_.count(ch)) {
            DBG_PRINTF("[PWM] Unknown channel %d\n", ch);
            return false;
        }
        return true;
    }

    std::map<int,int>   channelToPin_;
    std::map<int,int>   channelToLedc_;
    std::map<int,float> freqDefault_;
};
