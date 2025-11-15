#pragma once
#include <cstdint>

enum class EventType : uint8_t {
    HEARTBEAT = 0,
    PING,
    PONG,
    WHOMAI_REQUEST,
};

struct EventPayload {
    int32_t  i32 = 0;
    float    f32 = 0.0f;
    uint8_t  u8  = 0;
};

struct Event {
    EventType    type;
    uint32_t     timestamp_ms;  // millis()
    EventPayload payload;
};
