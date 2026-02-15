// include/config/CanDefs.h
// CAN bus message definitions for hybrid real-time/protocol transport
//
// Message ID Allocation (11-bit standard IDs):
//   0x000-0x0FF: Real-time control (highest priority)
//   0x100-0x1FF: Sensor feedback
//   0x200-0x2FF: Status/telemetry
//   0x300-0x3FF: Protocol transport (JSON wrapping)
//   0x400-0x4FF: Configuration/debug
//   0x500-0x7FF: Reserved for future use/extended IDs

#pragma once

#include <cstdint>
#include <cstring>

namespace can {

// =============================================================================
// NODE ADDRESSING
// =============================================================================

// Each node has a 4-bit ID (0-15), embedded in message IDs
// Example: Motor node 1 velocity command = 0x010 | (node_id << 0) = 0x011

constexpr uint8_t MAX_NODE_ID = 15;
constexpr uint8_t BROADCAST_ID = 0x0F;  // Broadcast to all nodes

// =============================================================================
// MESSAGE ID DEFINITIONS
// =============================================================================

namespace MsgId {
    // --- Real-time Control (0x000-0x0FF) - Highest priority ---
    constexpr uint16_t ESTOP           = 0x000;  // Emergency stop (broadcast)
    constexpr uint16_t SYNC            = 0x001;  // Sync pulse (broadcast)
    constexpr uint16_t HEARTBEAT_BASE  = 0x010;  // + node_id: Heartbeat
    constexpr uint16_t SET_VEL_BASE    = 0x020;  // + node_id: Set velocity
    constexpr uint16_t SET_SIGNAL_BASE = 0x030;  // + node_id: Set signal
    constexpr uint16_t STOP_BASE       = 0x040;  // + node_id: Stop command
    constexpr uint16_t ARM_BASE        = 0x050;  // + node_id: Arm
    constexpr uint16_t DISARM_BASE     = 0x060;  // + node_id: Disarm

    // --- Sensor Feedback (0x100-0x1FF) ---
    constexpr uint16_t ENCODER_BASE    = 0x100;  // + node_id: Encoder counts
    constexpr uint16_t IMU_ACCEL_BASE  = 0x110;  // + node_id: IMU accelerometer
    constexpr uint16_t IMU_GYRO_BASE   = 0x120;  // + node_id: IMU gyroscope
    constexpr uint16_t ANALOG_BASE     = 0x130;  // + node_id: Analog readings

    // --- Status/Telemetry (0x200-0x2FF) ---
    constexpr uint16_t STATUS_BASE     = 0x200;  // + node_id: Node status
    constexpr uint16_t ERROR_BASE      = 0x210;  // + node_id: Error report
    constexpr uint16_t TELEM_BASE      = 0x220;  // + node_id: Telemetry packet

    // --- Protocol Transport (0x300-0x3FF) ---
    // Multi-frame protocol for JSON commands
    constexpr uint16_t PROTO_CMD_BASE  = 0x300;  // + node_id: Command frames
    constexpr uint16_t PROTO_RSP_BASE  = 0x310;  // + node_id: Response frames
    constexpr uint16_t PROTO_ACK_BASE  = 0x320;  // + node_id: Acknowledgment

    // --- Configuration (0x400-0x4FF) ---
    constexpr uint16_t CONFIG_BASE     = 0x400;  // + node_id: Config messages
    constexpr uint16_t IDENTIFY_BASE   = 0x410;  // + node_id: Node identification
}

// Helper to build message ID with node
inline constexpr uint16_t makeId(uint16_t base, uint8_t nodeId) {
    return base | (nodeId & 0x0F);
}

// Extract node ID from message ID
inline constexpr uint8_t extractNodeId(uint16_t msgId) {
    return msgId & 0x0F;
}

// =============================================================================
// PACKED MESSAGE STRUCTURES
// =============================================================================

#pragma pack(push, 1)

// --- Control Messages (8 bytes max) ---

struct SetVelMsg {
    int16_t vx_mm_s;      // Linear velocity in mm/s (±32767)
    int16_t omega_mrad_s; // Angular velocity in mrad/s (±32767)
    uint16_t flags;       // Reserved flags
    uint16_t seq;         // Sequence number

    static constexpr float VX_SCALE = 1000.0f;    // mm/s
    static constexpr float OMEGA_SCALE = 1000.0f; // mrad/s

    void fromFloats(float vx, float omega) {
        vx_mm_s = static_cast<int16_t>(vx * VX_SCALE);
        omega_mrad_s = static_cast<int16_t>(omega * OMEGA_SCALE);
    }

    void toFloats(float& vx, float& omega) const {
        vx = static_cast<float>(vx_mm_s) / VX_SCALE;
        omega = static_cast<float>(omega_mrad_s) / OMEGA_SCALE;
    }
};
static_assert(sizeof(SetVelMsg) == 8, "SetVelMsg must be 8 bytes");

struct SetSignalMsg {
    uint16_t signal_id;   // Signal bus ID
    float value;          // Signal value (IEEE 754)
    uint16_t reserved;

    static size_t encode(uint16_t id, float val, uint8_t* buf) {
        SetSignalMsg msg{id, val, 0};
        memcpy(buf, &msg, sizeof(msg));
        return sizeof(msg);
    }
};
static_assert(sizeof(SetSignalMsg) == 8, "SetSignalMsg must be 8 bytes");

struct HeartbeatMsg {
    uint32_t uptime_ms;   // Uptime in milliseconds
    uint8_t state;        // Node state (see NodeState enum)
    uint8_t load_pct;     // CPU load percentage
    uint16_t errors;      // Error count since boot
};
static_assert(sizeof(HeartbeatMsg) == 8, "HeartbeatMsg must be 8 bytes");

// --- Sensor Messages ---

struct EncoderMsg {
    int32_t counts;       // Encoder counts (signed)
    int16_t velocity;     // Counts per second (clamped to ±32767)
    uint16_t timestamp;   // Local timestamp (ms, wraps at 65535)
};
static_assert(sizeof(EncoderMsg) == 8, "EncoderMsg must be 8 bytes");

struct ImuAccelMsg {
    int16_t ax;           // Accelerometer X (mg, ±32767 = ±32g)
    int16_t ay;           // Accelerometer Y
    int16_t az;           // Accelerometer Z
    uint16_t timestamp;
};
static_assert(sizeof(ImuAccelMsg) == 8, "ImuAccelMsg must be 8 bytes");

struct ImuGyroMsg {
    int16_t gx;           // Gyroscope X (mdps, ±32767 = ±32768 dps)
    int16_t gy;           // Gyroscope Y
    int16_t gz;           // Gyroscope Z
    uint16_t timestamp;
};
static_assert(sizeof(ImuGyroMsg) == 8, "ImuGyroMsg must be 8 bytes");

// --- Status Messages ---

enum class NodeState : uint8_t {
    INIT       = 0,
    IDLE       = 1,
    ARMED      = 2,
    ACTIVE     = 3,
    ERROR      = 4,
    ESTOPPED   = 5,
    RECOVERING = 6
};

struct StatusMsg {
    uint8_t state;        // NodeState enum
    uint8_t armed : 1;
    uint8_t active : 1;
    uint8_t estopped : 1;
    uint8_t error : 1;
    uint8_t reserved : 4;
    uint16_t voltage_mv;  // System voltage in mV
    uint16_t temp_c10;    // Temperature in 0.1°C
    uint16_t seq;         // Status sequence number
};
static_assert(sizeof(StatusMsg) == 8, "StatusMsg must be 8 bytes");

// --- Protocol Transport (for JSON wrapping) ---

// Protocol frame header
struct ProtoFrameHeader {
    uint8_t frame_id : 4;   // Frame sequence (0-15)
    uint8_t total_frames : 4; // Total frames in message (1-16)
    uint8_t msg_id;         // Message identifier for reassembly
    // Followed by 6 bytes of payload
};

constexpr size_t PROTO_PAYLOAD_SIZE = 6;  // Bytes per frame after header
constexpr size_t PROTO_MAX_FRAMES = 16;   // Max frames per message
constexpr size_t PROTO_MAX_MSG_SIZE = PROTO_PAYLOAD_SIZE * PROTO_MAX_FRAMES; // 96 bytes

struct ProtoFrame {
    ProtoFrameHeader header;
    uint8_t payload[PROTO_PAYLOAD_SIZE];
};
static_assert(sizeof(ProtoFrame) == 8, "ProtoFrame must be 8 bytes");

#pragma pack(pop)

// =============================================================================
// ENCODING/DECODING HELPERS
// =============================================================================

// Encode velocity command to CAN data
inline size_t encodeSetVel(float vx, float omega, uint16_t seq, uint8_t* data) {
    SetVelMsg msg;
    msg.fromFloats(vx, omega);
    msg.flags = 0;
    msg.seq = seq;
    memcpy(data, &msg, sizeof(msg));
    return sizeof(msg);
}

// Decode velocity command from CAN data
inline bool decodeSetVel(const uint8_t* data, size_t len, float& vx, float& omega) {
    if (len < sizeof(SetVelMsg)) return false;
    SetVelMsg msg;
    memcpy(&msg, data, sizeof(msg));
    msg.toFloats(vx, omega);
    return true;
}

// Encode encoder feedback
inline size_t encodeEncoder(int32_t counts, int16_t velocity, uint16_t timestamp, uint8_t* data) {
    EncoderMsg msg{counts, velocity, timestamp};
    memcpy(data, &msg, sizeof(msg));
    return sizeof(msg);
}

// Encode heartbeat
inline size_t encodeHeartbeat(uint32_t uptime, NodeState state, uint8_t load, uint16_t errors, uint8_t* data) {
    HeartbeatMsg msg{uptime, static_cast<uint8_t>(state), load, errors};
    memcpy(data, &msg, sizeof(msg));
    return sizeof(msg);
}

} // namespace can
