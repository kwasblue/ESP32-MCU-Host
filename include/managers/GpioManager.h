#pragma once
#include <Arduino.h>
#include "core/Debug.h"     // <-- add this

class GpioManager {
public:
    static constexpr int MAX_CHANNELS = 16;   // bump if you need more

    GpioManager() {
        for (int i = 0; i < MAX_CHANNELS; ++i) {
            pinForChannel_[i] = -1;
        }
    }

    void registerChannel(int ch, int pin, int mode = OUTPUT) {
        if (ch < 0 || ch >= MAX_CHANNELS) {
            DBG_PRINTF("[GPIO] registerChannel: invalid ch=%d (max=%d)\n",
                       ch, MAX_CHANNELS - 1);
            return;
        }

        DBG_PRINTF("[GPIO] registerChannel: ch=%d pin=%d mode=%d\n",
                   ch, pin, mode);

        pinMode(pin, mode);
        pinForChannel_[ch] = static_cast<int8_t>(pin);
    }

    bool hasChannel(int ch) const {
        if (ch < 0 || ch >= MAX_CHANNELS) {
            return false;
        }
        return pinForChannel_[ch] != -1;
    }

    void write(int ch, int value) {
        if (!hasChannel(ch)) {
            DBG_PRINTF("[GPIO] write: Unknown channel %d\n", ch);
            return;
        }
        uint8_t pin = static_cast<uint8_t>(pinForChannel_[ch]);
        digitalWrite(pin, value ? HIGH : LOW);
    }

    int read(int ch) const {
        if (!hasChannel(ch)) {
            DBG_PRINTF("[GPIO] read: Unknown channel %d\n", ch);
            return -1; // sentinel for "unknown"
        }
        uint8_t pin = static_cast<uint8_t>(pinForChannel_[ch]);
        return digitalRead(pin);
    }

    void toggle(int ch) {
        if (!hasChannel(ch)) {
            DBG_PRINTF("[GPIO] toggle: Unknown channel %d\n", ch);
            return;
        }
        uint8_t pin = static_cast<uint8_t>(pinForChannel_[ch]);
        int current = digitalRead(pin);
        digitalWrite(pin, current == HIGH ? LOW : HIGH);
    }

    void GpioManager::configureLimitSwitch(uint8_t pin) {
        pinMode(pin, INPUT_PULLUP);
    }
    
private:
    int8_t pinForChannel_[MAX_CHANNELS];
};
