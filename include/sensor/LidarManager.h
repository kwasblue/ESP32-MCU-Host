#pragma once

#include "config/FeatureFlags.h"

#if HAS_LIDAR

#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

class LidarManager {
public:
    struct Sample {
        float distance_m = NAN;
    };

    // Default I2C address for VL53L0X
    static constexpr uint8_t DEFAULT_ADDR = 0x29;

    LidarManager() = default;

    // Same style as ImuManager::begin(...)
    bool begin(uint8_t sdaPin, uint8_t sclPin, uint8_t addr = DEFAULT_ADDR);

    bool isOnline() const { return online_; }

    // Same style as ImuManager::readSample(...)
    bool readSample(Sample& out);

private:
    bool    online_ = false;
    uint8_t addr_   = DEFAULT_ADDR;

    VL53L0X lidar_;
};

#else // !HAS_LIDAR

#include <cmath>

// Stub when LiDAR is disabled
class LidarManager {
public:
    struct Sample {
        float distance_m = NAN;
    };

    static constexpr uint8_t DEFAULT_ADDR = 0x29;

    LidarManager() = default;
    bool begin(uint8_t, uint8_t, uint8_t = DEFAULT_ADDR) { return false; }
    bool isOnline() const { return false; }
    bool readSample(Sample&) { return false; }
};

#endif // HAS_LIDAR
