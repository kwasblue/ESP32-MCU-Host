// include/managers/EncoderManager.h
#pragma once

#include <Arduino.h>

// Simple quadrature encoder manager (A/B only, no index/Z yet).
// - Supports up to MAX_ENCODERS encoders
// - Uses ISR callbacks wired in EncoderManager.cpp
// - Count is signed: positive / negative based on direction
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

    // Attach an encoder to A/B pins and install ISR(s).
    // id must be < MAX_ENCODERS.
    void attach(uint8_t id, uint8_t pinA, uint8_t pinB);

    // Get current tick count (signed). Safe-ish snapshot.
    int32_t getCount(uint8_t id) const;

    // Reset tick count to 0.
    void reset(uint8_t id);

    // ISR helpers called from static ISRs:
    void handleA(uint8_t id);
    void handleB(uint8_t id);

    // Optional: simple presence check
    bool isAttached(uint8_t id) const {
        return (id < MAX_ENCODERS) && encoders_[id].initialized;
    }

private:
    Encoder encoders_[MAX_ENCODERS];
};

// Global pointer used by file-local ISRs to call into the manager.
EncoderManager* getGlobalEncoderManager();
