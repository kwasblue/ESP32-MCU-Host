// AUTO-GENERATED FILE — DO NOT EDIT BY HAND
// Generated from COMMANDS in platform_schema.py

#pragma once
#include <string>

enum class CmdType {
    CLEAR_ESTOP,
    ESTOP,
    GPIO_READ,
    GPIO_REGISTER_CHANNEL,
    GPIO_TOGGLE,
    GPIO_WRITE,
    LED_OFF,
    LED_ON,
    PWM_SET,
    SERVO_ATTACH,
    SERVO_DETACH,
    SERVO_SET_ANGLE,
    SET_LOG_LEVEL,
    SET_MODE,
    SET_VEL,
    STEPPER_ENABLE,
    STEPPER_MOVE_REL,
    STEPPER_STOP,
    STOP,
    TELEM_SET_INTERVAL,
    ULTRASONIC_ATTACH,
    ULTRASONIC_READ,
    UNKNOWN
};

inline CmdType cmdTypeFromString(const std::string& s) {
    if (s == "CMD_CLEAR_ESTOP") return CmdType::CLEAR_ESTOP;
    if (s == "CMD_ESTOP") return CmdType::ESTOP;
    if (s == "CMD_GPIO_READ") return CmdType::GPIO_READ;
    if (s == "CMD_GPIO_REGISTER_CHANNEL") return CmdType::GPIO_REGISTER_CHANNEL;
    if (s == "CMD_GPIO_TOGGLE") return CmdType::GPIO_TOGGLE;
    if (s == "CMD_GPIO_WRITE") return CmdType::GPIO_WRITE;
    if (s == "CMD_LED_OFF") return CmdType::LED_OFF;
    if (s == "CMD_LED_ON") return CmdType::LED_ON;
    if (s == "CMD_PWM_SET") return CmdType::PWM_SET;
    if (s == "CMD_SERVO_ATTACH") return CmdType::SERVO_ATTACH;
    if (s == "CMD_SERVO_DETACH") return CmdType::SERVO_DETACH;
    if (s == "CMD_SERVO_SET_ANGLE") return CmdType::SERVO_SET_ANGLE;
    if (s == "CMD_SET_LOG_LEVEL") return CmdType::SET_LOG_LEVEL;
    if (s == "CMD_SET_MODE") return CmdType::SET_MODE;
    if (s == "CMD_SET_VEL") return CmdType::SET_VEL;
    if (s == "CMD_STEPPER_ENABLE") return CmdType::STEPPER_ENABLE;
    if (s == "CMD_STEPPER_MOVE_REL") return CmdType::STEPPER_MOVE_REL;
    if (s == "CMD_STEPPER_STOP") return CmdType::STEPPER_STOP;
    if (s == "CMD_STOP") return CmdType::STOP;
    if (s == "CMD_TELEM_SET_INTERVAL") return CmdType::TELEM_SET_INTERVAL;
    if (s == "CMD_ULTRASONIC_ATTACH") return CmdType::ULTRASONIC_ATTACH;
    if (s == "CMD_ULTRASONIC_READ") return CmdType::ULTRASONIC_READ;
    return CmdType::UNKNOWN;
}

inline const char* cmdTypeToString(CmdType c) {
    switch (c) {
        case CmdType::CLEAR_ESTOP: return "CMD_CLEAR_ESTOP";
        case CmdType::ESTOP: return "CMD_ESTOP";
        case CmdType::GPIO_READ: return "CMD_GPIO_READ";
        case CmdType::GPIO_REGISTER_CHANNEL: return "CMD_GPIO_REGISTER_CHANNEL";
        case CmdType::GPIO_TOGGLE: return "CMD_GPIO_TOGGLE";
        case CmdType::GPIO_WRITE: return "CMD_GPIO_WRITE";
        case CmdType::LED_OFF: return "CMD_LED_OFF";
        case CmdType::LED_ON: return "CMD_LED_ON";
        case CmdType::PWM_SET: return "CMD_PWM_SET";
        case CmdType::SERVO_ATTACH: return "CMD_SERVO_ATTACH";
        case CmdType::SERVO_DETACH: return "CMD_SERVO_DETACH";
        case CmdType::SERVO_SET_ANGLE: return "CMD_SERVO_SET_ANGLE";
        case CmdType::SET_LOG_LEVEL: return "CMD_SET_LOG_LEVEL";
        case CmdType::SET_MODE: return "CMD_SET_MODE";
        case CmdType::SET_VEL: return "CMD_SET_VEL";
        case CmdType::STEPPER_ENABLE: return "CMD_STEPPER_ENABLE";
        case CmdType::STEPPER_MOVE_REL: return "CMD_STEPPER_MOVE_REL";
        case CmdType::STEPPER_STOP: return "CMD_STEPPER_STOP";
        case CmdType::STOP: return "CMD_STOP";
        case CmdType::TELEM_SET_INTERVAL: return "CMD_TELEM_SET_INTERVAL";
        case CmdType::ULTRASONIC_ATTACH: return "CMD_ULTRASONIC_ATTACH";
        case CmdType::ULTRASONIC_READ: return "CMD_ULTRASONIC_READ";
        default: return "UNKNOWN";
    }
}
