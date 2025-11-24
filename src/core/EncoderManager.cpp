// src/managers/EncoderManager.cpp
#include "managers/EncoderManager.h"
#include "core/Debug.h"

// Single global instance pointer (since you already use a single g_encoder).
// The constructor will register itself here.
static EncoderManager* s_globalEncoderMgr = nullptr;

// Exposed so other files (e.g. main.cpp) can query if needed.
EncoderManager* getGlobalEncoderManager() {
    return s_globalEncoderMgr;
}

EncoderManager::EncoderManager() {
    s_globalEncoderMgr = this;
}

// ---- Internal helpers ----

// On ESP32 you *can* use CHANGE on A only and read B inside the ISR.
// That’s what we do here. We also install a B ISR so we could upgrade
// to full 4x decoding later if you want.
void EncoderManager::attach(uint8_t id, uint8_t pinA, uint8_t pinB) {
    if (id >= MAX_ENCODERS) {
        DBG_PRINTF("[EncoderManager] attach: id=%u out of range\n", id);
        return;
    }

    Encoder& e = encoders_[id];

    e.pinA = pinA;
    e.pinB = pinB;
    e.count = 0;
    e.initialized = true;

    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);

    DBG_PRINTF("[EncoderManager] attach id=%u pinA=%u pinB=%u\n",
               id, pinA, pinB);

    // Install interrupts (see static ISRs below).
    // We trigger on CHANGE on A, and optionally on B too.
    switch (id) {
        case 0:
            attachInterrupt(digitalPinToInterrupt(pinA), []() {
                if (s_globalEncoderMgr) s_globalEncoderMgr->handleA(0);
            }, CHANGE);
            attachInterrupt(digitalPinToInterrupt(pinB), []() {
                if (s_globalEncoderMgr) s_globalEncoderMgr->handleB(0);
            }, CHANGE);
            break;

        case 1:
            attachInterrupt(digitalPinToInterrupt(pinA), []() {
                if (s_globalEncoderMgr) s_globalEncoderMgr->handleA(1);
            }, CHANGE);
            attachInterrupt(digitalPinToInterrupt(pinB), []() {
                if (s_globalEncoderMgr) s_globalEncoderMgr->handleB(1);
            }, CHANGE);
            break;

        default:
            // If you ever bump MAX_ENCODERS and forget to add here:
            DBG_PRINTF("[EncoderManager] attach: no ISR wiring for id=%u\n", id);
            break;
    }
}

int32_t EncoderManager::getCount(uint8_t id) const {
    if (id >= MAX_ENCODERS || !encoders_[id].initialized) {
        return 0;
    }

    // Take an atomic-ish snapshot. On ESP32, reading a 32-bit volatile is
    // already atomic, but this guards against weirdness if you want.
    noInterrupts();
    int32_t c = encoders_[id].count;
    interrupts();
    return c;
}

void EncoderManager::reset(uint8_t id) {
    if (id >= MAX_ENCODERS || !encoders_[id].initialized) {
        return;
    }

    noInterrupts();
    encoders_[id].count = 0;
    interrupts();
}

// ---- ISR helpers ----

// Quadrature rule used:
//   On a change of A or B, read both pins and decide direction.
//   A == B  -> +1
//   A != B  -> -1
//
// We call the same logic from handleA and handleB, so you get up to 4x decoding.
void EncoderManager::handleA(uint8_t id) {
    if (id >= MAX_ENCODERS || !encoders_[id].initialized) {
        return;
    }
    Encoder& e = encoders_[id];

    int a = digitalRead(e.pinA);
    int b = digitalRead(e.pinB);

    int dir = (a == b) ? +1 : -1;
    e.count += dir;
}

void EncoderManager::handleB(uint8_t id) {
    if (id >= MAX_ENCODERS || !encoders_[id].initialized) {
        return;
    }
    Encoder& e = encoders_[id];

    int a = digitalRead(e.pinA);
    int b = digitalRead(e.pinB);

    int dir = (a == b) ? +1 : -1;
    e.count += dir;
}
