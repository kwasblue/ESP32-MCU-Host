// include/core/SignalBus.h
// Signal routing system for control kernel
// Stores named signals with values and timestamps

#pragma once

#include <cstdint>
#include <vector>
#include <cstring>

class SignalBus {
public:
    enum class Kind : uint8_t { 
        REF  = 0,   // Reference/setpoint
        MEAS = 1,   // Measurement/feedback
        OUT  = 2    // Control output
    };

    // Fixed-size name buffer to avoid dangling pointer issues with JSON strings
    static constexpr size_t NAME_MAX_LEN = 31;

    struct SignalDef {
        uint16_t id = 0;
        char name[NAME_MAX_LEN + 1] = {0};  // Fixed buffer, null-terminated
        Kind kind = Kind::REF;
        float value = 0.0f;
        uint32_t ts_ms = 0;
    };

    // Define a new signal (returns false if ID already exists)
    bool define(uint16_t id, const char* name, Kind kind, float initial = 0.0f);
    
    // Check if signal exists
    bool exists(uint16_t id) const;
    
    // Set signal value with timestamp
    bool set(uint16_t id, float v, uint32_t now_ms = 0);
    
    // Get signal value
    bool get(uint16_t id, float& out) const;
    
    // Get signal timestamp
    bool getTimestamp(uint16_t id, uint32_t& out) const;
    
    // Get signal by ID (returns nullptr if not found)
    const SignalDef* find(uint16_t id) const;
    
    // Access all signals (for telemetry/listing)
    const std::vector<SignalDef>& all() const { return signals_; }
    
    // Number of defined signals
    size_t count() const { return signals_.size(); }
    
    // Clear all signals
    void clear() { signals_.clear(); }

private:
    std::vector<SignalDef> signals_;
    int indexOf_(uint16_t id) const;
};

// Helper to convert Kind to string
inline const char* signalKindToString(SignalBus::Kind k) {
    switch (k) {
        case SignalBus::Kind::REF:  return "REF";
        case SignalBus::Kind::MEAS: return "MEAS";
        case SignalBus::Kind::OUT:  return "OUT";
        default: return "UNK";
    }
}

// Helper to parse Kind from string
inline SignalBus::Kind signalKindFromString(const char* s) {
    if (!s) return SignalBus::Kind::REF;
    if (strcmp(s, "REF") == 0)  return SignalBus::Kind::REF;
    if (strcmp(s, "MEAS") == 0) return SignalBus::Kind::MEAS;
    if (strcmp(s, "OUT") == 0)  return SignalBus::Kind::OUT;
    return SignalBus::Kind::REF;
}