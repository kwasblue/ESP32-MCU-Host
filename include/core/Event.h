#pragma once
#include <cstdint>
#include <string>

enum class EventType : uint8_t {
    HEARTBEAT = 0,
    PING,
    PONG,
    WHOMAI_REQUEST,
    JSON_MESSAGE_RX,
    JSON_MESSAGE_TX,
    TELEMETRY_SYSTEM,
    TELEMETRY_MOTION,
    TELEMETRY_SERVOS,
};

struct TelemetrySystemPayload {
    uint32_t uptime_ms;
    uint32_t free_heap;
};

struct TelemetryMotionPayload {
    float vx;        // m/s
    float omega;     // rad/s
    float left_cmd;  // whatever DcMotorManager uses (duty/rpm/etc.)
    float right_cmd;
};

constexpr uint8_t MAX_TELEMETRY_SERVOS = 8;

struct TelemetryServosPayload {
    uint8_t num_channels;
    float   angles[MAX_TELEMETRY_SERVOS];
};


struct EventPayload {
    int32_t  i32 = 0;
    float    f32 = 0.0f;
    uint8_t  u8  = 0;

    std::string json;
    // --- telemetry payloads ---
    TelemetrySystemPayload telemSystem{};
    TelemetryMotionPayload telemMotion{};
    TelemetryServosPayload telemServos{};

};

struct Event {
    EventType    type;
    uint32_t     timestamp_ms;  // millis()
    EventPayload payload;
};
