// core/Messages.h
#pragma once
#include <cstdint>

enum class MsgType : uint8_t {
    PING        = 0x01,
    PONG        = 0x02,
    HEARTBEAT   = 0x03,

    WHOAMI      = 0x10,  // host -> robot (request identity)
    HELLO       = 0x11,  // robot -> host (identity response)

    STATUS      = 0x20,  // robot -> host (telemetry snapshot)

    LOG         = 0x30,  // robot -> host (log line)

    SET_PARAM   = 0x40,  // host -> robot (change config param)
    GET_PARAM   = 0x41,  // host -> robot (request param)
    PARAM_VALUE = 0x42,  // robot -> host (param response)

    ERROR       = 0x7F,  // robot -> host (error with code)
};

