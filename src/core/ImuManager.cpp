#include "managers/ImuManager.h"

// MPU-60x0 / MPU-9250 style registers
static constexpr uint8_t REG_PWR_MGMT_1    = 0x6B;
static constexpr uint8_t REG_WHO_AM_I      = 0x75;
static constexpr uint8_t REG_ACCEL_XOUT_H  = 0x3B;

// WHO_AM_I expected values (depends on chip, keep both)
static constexpr uint8_t WHO_AM_I_MPU6050  = 0x68;
static constexpr uint8_t WHO_AM_I_MPU6500  = 0x70; // some variants
static constexpr uint8_t WHO_AM_I_MPU9250  = 0x71;

bool ImuManager::begin(int sdaPin, int sclPin, uint8_t addr) {
    addr_ = addr;

    DBG_PRINTF("[ImuManager] begin() SDA=%d SCL=%d addr=0x%02X\n", sdaPin, sclPin, addr_);

    // Initialize I2C on given pins
    Wire.begin(sdaPin, sclPin);
    Wire.setClock(400000);  // 400 kHz

    // Read WHO_AM_I
    uint8_t who = readByte(REG_WHO_AM_I);
    DBG_PRINTF("[ImuManager] WHO_AM_I=0x%02X\n", who);

    if (who != WHO_AM_I_MPU6050 &&
        who != WHO_AM_I_MPU6500 &&
        who != WHO_AM_I_MPU9250)
    {
        DBG_PRINTLN("[ImuManager] Unexpected WHO_AM_I, IMU may not be connected");
        online_ = false;
        return false;
    }

    // Wake up device: clear sleep bit
    writeByte(REG_PWR_MGMT_1, 0x00);
    delay(100);

    DBG_PRINTLN("[ImuManager] IMU online and initialized");
    online_ = true;
    return true;
}

void ImuManager::writeByte(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(addr_);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

uint8_t ImuManager::readByte(uint8_t reg) {
    Wire.beginTransmission(addr_);
    Wire.write(reg);
    Wire.endTransmission(false);  // repeated start

    Wire.requestFrom(addr_, (uint8_t)1);
    if (Wire.available()) {
        return Wire.read();
    }
    return 0;
}

bool ImuManager::readBytes(uint8_t startReg, uint8_t* buffer, size_t length) {
    Wire.beginTransmission(addr_);
    Wire.write(startReg);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    size_t readCount = Wire.requestFrom(addr_, (uint8_t)length);
    if (readCount != length) {
        return false;
    }

    for (size_t i = 0; i < length; ++i) {
        if (!Wire.available()) {
            return false;
        }
        buffer[i] = Wire.read();
    }
    return true;
}

bool ImuManager::readSample(Sample& out) {
    if (!online_) {
        return false;
    }

    uint8_t raw[14];
    if (!readBytes(REG_ACCEL_XOUT_H, raw, sizeof(raw))) {
        DBG_PRINTLN("[ImuManager] readBytes failed");
        return false;
    }

    auto toInt16 = [](uint8_t hi, uint8_t lo) -> int16_t {
        return (int16_t)((hi << 8) | lo);
    };

    int16_t ax_raw   = toInt16(raw[0],  raw[1]);
    int16_t ay_raw   = toInt16(raw[2],  raw[3]);
    int16_t az_raw   = toInt16(raw[4],  raw[5]);
    int16_t temp_raw = toInt16(raw[6],  raw[7]);
    int16_t gx_raw   = toInt16(raw[8],  raw[9]);
    int16_t gy_raw   = toInt16(raw[10], raw[11]);
    int16_t gz_raw   = toInt16(raw[12], raw[13]);

    // Convert to physical units (assuming default config):
    // accel: ±2g → 16384 LSB/g
    // gyro:  ±250 deg/s → 131 LSB/(deg/s)
    out.ax_g   = ax_raw / 16384.0f;
    out.ay_g   = ay_raw / 16384.0f;
    out.az_g   = az_raw / 16384.0f;

    out.gx_dps = gx_raw / 131.0f;
    out.gy_dps = gy_raw / 131.0f;
    out.gz_dps = gz_raw / 131.0f;

    // Temperature for MPU-6050: Temp(°C) = temp_raw/340 + 36.53
    out.temp_c = (temp_raw / 340.0f) + 36.53f;

    return true;
}
