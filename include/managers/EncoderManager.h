#pragma once
#include <Arduino.h>

class EncoderManager {
public:
    struct Encoder {
        volatile int32_t count      = 0;
        uint8_t          pinA       = 0;
        uint8_t          pinB       = 0;
        bool             initialized = false;
    };

    static constexpr uint8_t MAX_ENCODERS = 2;

    EncoderManager();

    void    attach(uint8_t id, uint8_t pinA, uint8_t pinB);
    int32_t getCount(uint8_t id) const;
    void    reset(uint8_t id);

    // later: real ISR hooks
    void handleA(uint8_t id);
    void handleB(uint8_t id);

private:
    Encoder encoders_[MAX_ENCODERS];
};
