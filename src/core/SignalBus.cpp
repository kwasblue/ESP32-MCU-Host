// src/core/SignalBus.cpp
// Signal routing system implementation

#include "core/SignalBus.h"
#include <cstring>

int SignalBus::indexOf_(uint16_t id) const {
    for (size_t i = 0; i < signals_.size(); i++) {
        if (signals_[i].id == id) return static_cast<int>(i);
    }
    return -1;
}

bool SignalBus::define(uint16_t id, const char* name, Kind kind, float initial) {
    // Don't allow duplicate IDs
    if (exists(id)) return false;
    
    SignalDef d;
    d.id = id;
    d.kind = kind;
    d.value = initial;
    d.ts_ms = 0;
    
    // Safe string copy - prevents dangling pointer when JSON goes out of scope
    if (name && name[0] != '\0') {
        strncpy(d.name, name, NAME_MAX_LEN);
        d.name[NAME_MAX_LEN] = '\0';  // Ensure null termination
    } else {
        d.name[0] = '\0';
    }
    
    signals_.push_back(d);
    return true;
}

bool SignalBus::exists(uint16_t id) const {
    return indexOf_(id) >= 0;
}

bool SignalBus::set(uint16_t id, float v, uint32_t now_ms) {
    int idx = indexOf_(id);
    if (idx < 0) return false;
    signals_[static_cast<size_t>(idx)].value = v;
    signals_[static_cast<size_t>(idx)].ts_ms = now_ms;
    return true;
}

bool SignalBus::get(uint16_t id, float& out) const {
    int idx = indexOf_(id);
    if (idx < 0) return false;
    out = signals_[static_cast<size_t>(idx)].value;
    return true;
}

bool SignalBus::getTimestamp(uint16_t id, uint32_t& out) const {
    int idx = indexOf_(id);
    if (idx < 0) return false;
    out = signals_[static_cast<size_t>(idx)].ts_ms;
    return true;
}

const SignalBus::SignalDef* SignalBus::find(uint16_t id) const {
    int idx = indexOf_(id);
    if (idx < 0) return nullptr;
    return &signals_[static_cast<size_t>(idx)];
}