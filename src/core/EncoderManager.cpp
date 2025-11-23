#include "managers/EncoderManager.h"

EncoderManager::EncoderManager() {
    for (auto &e : encoders_) {
        e.count       = 0;
        e.pinA        = 0;
        e.pinB        = 0;
        e.initialized = false;
    }
}

void EncoderManager::attach(uint8_t id, uint8_t pinA, uint8_t pinB) {
    if (id >= MAX_ENCODERS) return;

    Encoder &enc = encoders_[id];
    enc.pinA        = pinA;
    enc.pinB        = pinB;
    enc.count       = 0;
    enc.initialized = true;

    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);

    // TODO: attachInterrupt(digitalPinToInterrupt(pinA), ...);
    // TODO: attachInterrupt(digitalPinToInterrupt(pinB), ...);
}

int32_t EncoderManager::getCount(uint8_t id) const {
    if (id >= MAX_ENCODERS) return 0;
    const Encoder &enc = encoders_[id];
    if (!enc.initialized) return 0;
    return enc.count;
}

void EncoderManager::reset(uint8_t id) {
    if (id >= MAX_ENCODERS) return;
    encoders_[id].count = 0;
}

void EncoderManager::handleA(uint8_t id) {
    if (id >= MAX_ENCODERS) return;
    // TODO: proper quadrature decode; for now just increment
    encoders_[id].count++;
}

void EncoderManager::handleB(uint8_t id) {
    if (id >= MAX_ENCODERS) return;
    // TODO: proper quadrature decode; for now just decrement
    encoders_[id].count--;
}
