// AUTO-GENERATED FILE — DO NOT EDIT BY HAND
// Generated from GPIO_CHANNELS in platform_schema.py

#pragma once
#include <Arduino.h>
#include "config/PinConfig.h"

struct GpioChannelDef {
    int      channel;
    uint8_t  pin;
    uint8_t  mode;
    const char* name;
};

constexpr GpioChannelDef GPIO_CHANNEL_DEFS[] = {
    { 0, Pins::LED_STATUS, OUTPUT, "LED_STATUS" },
};

constexpr size_t GPIO_CHANNEL_COUNT = sizeof(GPIO_CHANNEL_DEFS) / sizeof(GPIO_CHANNEL_DEFS[0]);
