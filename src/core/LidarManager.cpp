#include "managers/LidarManager.h"
#include "core/Debug.h"

bool LidarManager::begin(uint8_t sdaPin, uint8_t sclPin, uint8_t addr) {
    addr_ = addr;

    Wire.begin(sdaPin, sclPin);

    if (!lidar_.init()) {
        DBG_PRINTLN("[LidarManager] VL53L0X init() failed");
        online_ = false;
        return false;
    }

    // If you ever want to change address (multiple sensors), you’d call setAddress(addr_) here.
    // lidar_.setAddress(addr_);

    lidar_.setTimeout(500);      // ms timeout for reads
    lidar_.startContinuous();    // continuous mode for fast polling

    online_ = true;
    DBG_PRINTLN("[LidarManager] VL53L0X initialized OK");
    return true;
}

bool LidarManager::readSample(Sample& out) {
    if (!online_) {
        return false;
    }

    uint16_t mm = lidar_.readRangeContinuousMillimeters();

    if (lidar_.timeoutOccurred()) {
        DBG_PRINTLN("[LidarManager] VL53L0X timeout");
        return false;
    }

    // Basic sanity check
    if (mm == 0 || mm > 4000) {   // ~4m cap
        return false;
    }

    out.distance_m = mm / 1000.0f;
    return true;
}
