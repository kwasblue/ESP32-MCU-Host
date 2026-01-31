// include/core/BinaryCommands.h
// Binary command protocol for high-rate streaming (10x smaller than JSON)
//
// Binary commands are compact fixed-format messages for control loops.
// Use JSON commands for setup/config, binary for real-time streaming.

#pragma once

#include <cstdint>
#include <cstring>

namespace BinaryCommands {

// Binary command opcodes
enum class Opcode : uint8_t {
    SET_VEL         = 0x10,  // Set velocity: vx(f32), omega(f32)
    SET_SIGNAL      = 0x11,  // Set signal: id(u16), value(f32)
    SET_SIGNALS     = 0x12,  // Set multiple signals: count(u8), [id(u16), value(f32)]*
    HEARTBEAT       = 0x20,  // Heartbeat (no payload)
    STOP            = 0x21,  // Emergency stop (no payload)
};

// -----------------------------------------------------------------------------
// Command Structures (POD types for union compatibility)
// -----------------------------------------------------------------------------

struct SetVelCmd {
    float vx;
    float omega;
};

struct SetSignalCmd {
    uint16_t id;
    float value;
};

struct SetSignalsCmd {
    uint8_t count;
};

// -----------------------------------------------------------------------------
// Decode Result
// -----------------------------------------------------------------------------

struct DecodeResult {
    Opcode opcode;
    bool valid;
    SetVelCmd set_vel;
    SetSignalCmd set_signal;
    SetSignalsCmd set_signals;

    DecodeResult() : opcode(Opcode::HEARTBEAT), valid(false) {
        set_vel.vx = 0.0f;
        set_vel.omega = 0.0f;
        set_signal.id = 0;
        set_signal.value = 0.0f;
        set_signals.count = 0;
    }
};

// -----------------------------------------------------------------------------
// Decode Functions
// -----------------------------------------------------------------------------

// Helper to read little-endian values
inline uint16_t read_u16_le(const uint8_t* buf) {
    return static_cast<uint16_t>(buf[0]) | (static_cast<uint16_t>(buf[1]) << 8);
}

inline float read_f32_le(const uint8_t* buf) {
    uint32_t v = static_cast<uint32_t>(buf[0])
               | (static_cast<uint32_t>(buf[1]) << 8)
               | (static_cast<uint32_t>(buf[2]) << 16)
               | (static_cast<uint32_t>(buf[3]) << 24);
    float f;
    memcpy(&f, &v, sizeof(f));
    return f;
}

/**
 * Decode a binary command packet
 * @param data Pointer to command data (after opcode byte)
 * @param len Length of data
 * @param opcode Command opcode (first byte of packet)
 * @return DecodeResult with parsed command
 */
inline DecodeResult decode(const uint8_t* data, size_t len, uint8_t opcode) {
    DecodeResult result;
    result.valid = false;

    switch (static_cast<Opcode>(opcode)) {
    case Opcode::SET_VEL:
        if (len >= 8) {
            result.opcode = Opcode::SET_VEL;
            result.set_vel.vx = read_f32_le(data);
            result.set_vel.omega = read_f32_le(data + 4);
            result.valid = true;
        }
        break;

    case Opcode::SET_SIGNAL:
        if (len >= 6) {
            result.opcode = Opcode::SET_SIGNAL;
            result.set_signal.id = read_u16_le(data);
            result.set_signal.value = read_f32_le(data + 2);
            result.valid = true;
        }
        break;

    case Opcode::SET_SIGNALS:
        if (len >= 1) {
            result.opcode = Opcode::SET_SIGNALS;
            result.set_signals.count = data[0];
            // Validate we have enough data for all signals
            size_t needed = 1 + result.set_signals.count * 6;
            result.valid = (len >= needed);
        }
        break;

    case Opcode::HEARTBEAT:
        result.opcode = Opcode::HEARTBEAT;
        result.valid = true;
        break;

    case Opcode::STOP:
        result.opcode = Opcode::STOP;
        result.valid = true;
        break;

    default:
        break;
    }

    return result;
}

/**
 * Parse individual signals from SET_SIGNALS command
 * @param data Pointer to start of signal data (after count byte)
 * @param index Signal index (0 to count-1)
 * @param id_out Output signal ID
 * @param value_out Output signal value
 */
inline void parseSignal(const uint8_t* data, uint8_t index, uint16_t& id_out, float& value_out) {
    const uint8_t* ptr = data + (index * 6);
    id_out = read_u16_le(ptr);
    value_out = read_f32_le(ptr + 2);
}

// -----------------------------------------------------------------------------
// Encode Functions (for Python host reference)
// -----------------------------------------------------------------------------

/**
 * Encode SET_VEL command
 * @param vx Linear velocity
 * @param omega Angular velocity
 * @param buf Output buffer (must be at least 9 bytes)
 * @return Number of bytes written
 */
inline size_t encodeSetVel(float vx, float omega, uint8_t* buf) {
    buf[0] = static_cast<uint8_t>(Opcode::SET_VEL);

    uint32_t v;
    memcpy(&v, &vx, sizeof(v));
    buf[1] = v & 0xFF;
    buf[2] = (v >> 8) & 0xFF;
    buf[3] = (v >> 16) & 0xFF;
    buf[4] = (v >> 24) & 0xFF;

    memcpy(&v, &omega, sizeof(v));
    buf[5] = v & 0xFF;
    buf[6] = (v >> 8) & 0xFF;
    buf[7] = (v >> 16) & 0xFF;
    buf[8] = (v >> 24) & 0xFF;

    return 9;
}

/**
 * Encode SET_SIGNAL command
 * @param id Signal ID
 * @param value Signal value
 * @param buf Output buffer (must be at least 7 bytes)
 * @return Number of bytes written
 */
inline size_t encodeSetSignal(uint16_t id, float value, uint8_t* buf) {
    buf[0] = static_cast<uint8_t>(Opcode::SET_SIGNAL);
    buf[1] = id & 0xFF;
    buf[2] = (id >> 8) & 0xFF;

    uint32_t v;
    memcpy(&v, &value, sizeof(v));
    buf[3] = v & 0xFF;
    buf[4] = (v >> 8) & 0xFF;
    buf[5] = (v >> 16) & 0xFF;
    buf[6] = (v >> 24) & 0xFF;

    return 7;
}

} // namespace BinaryCommands
