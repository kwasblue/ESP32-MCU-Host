#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "core/Debug.h"

class ImuManager {
public:
    struct Sample {
        float ax_g = 0.0f;
        float ay_g = 0.0f;
        float az_g = 0.0f;
        float gx_dps = 0.0f;
        float gy_dps = 0.0f;
        float gz_dps = 0.0f;
        float temp_c = 0.0f;
    };

    ImuManager() = default;

    // Initialize I2C + IMU. Returns true if WHO_AM_I looks OK.
    bool begin(int sdaPin, int sclPin, uint8_t addr = 0x68);

    bool isOnline() const { return online_; }

    // Read one accel/gyro/temp sample. Returns false if IMU offline or read failed.
    bool readSample(Sample& out);

private:
    uint8_t  addr_   = 0x68;
    bool     online_ = false;

    // Low-level helpers
    void     writeByte(uint8_t reg, uint8_t value);
    uint8_t  readByte(uint8_t reg);
    bool     readBytes(uint8_t startReg, uint8_t* buffer, size_t length);
};
