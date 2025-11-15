// core/Messages.h
#pragma once
#include <cstdint>
#include <string>
#include <ArduinoJson.h>

enum class MsgType : uint8_t {
    // ---- Low-level transport primitives ----
    PING        = 0x01,
    PONG        = 0x02,
    HEARTBEAT   = 0x03,

    // ---- Identity / boot ----
    WHOAMI      = 0x10,
    HELLO       = 0x11,

    // ---- Telemetry ----
    STATUS      = 0x20,  // robot -> host (snapshot/telemetry)

    // ---- Logging ----
    LOG         = 0x30,  // robot -> host log line

    // ---- Low-level config ----
    SET_PARAM   = 0x40,  // host -> robot (update parameter)
    GET_PARAM   = 0x41,
    PARAM_VALUE = 0x42,

    // ---- NEW: High-level JSON command envelope ----
    CMD_JSON    = 0x50,  // host -> robot (structured HI-LEVEL COMMAND)

    // ---- Errors ----
    ERROR       = 0x7F,
};

// ---- High-level JSON classification ----
enum class MsgKind {
    CMD,
    TELEMETRY,
    EVENT,
    RESP,
    ERROR,
    UNKNOWN
};

inline MsgKind msgKindFromString(const std::string& s) {
    if (s == "cmd")        return MsgKind::CMD;
    if (s == "telemetry")  return MsgKind::TELEMETRY;
    if (s == "event")      return MsgKind::EVENT;
    if (s == "resp")       return MsgKind::RESP;
    if (s == "error")      return MsgKind::ERROR;
    return MsgKind::UNKNOWN;
}

// ---- Command types for robot side ----
enum class CmdType {
    SET_MODE,
    SET_VEL,
    STOP,
    ESTOP,
    CLEAR_ESTOP,
    UNKNOWN,
    LED_ON,         
    LED_OFF,       
};

inline CmdType cmdTypeFromString(const std::string& s) {
    if (s == "CMD_SET_MODE")    return CmdType::SET_MODE;
    if (s == "CMD_SET_VEL")     return CmdType::SET_VEL;
    if (s == "CMD_STOP")        return CmdType::STOP;
    if (s == "CMD_ESTOP")       return CmdType::ESTOP;
    if (s == "CMD_CLEAR_ESTOP") return CmdType::CLEAR_ESTOP;
    if (s == "CMD_LED_ON")      return CmdType::LED_ON;   
    if (s == "CMD_LED_OFF")     return CmdType::LED_OFF; 
    return CmdType::UNKNOWN;
}