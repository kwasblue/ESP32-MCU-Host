#include "setup/ISetupModule.h"
#include "core/ServiceContext.h"

#include <Arduino.h>
#include "config/PinConfig.h"
#include "sensor/ImuManager.h"
#include "sensor/LidarManager.h"

namespace {

class SetupSensorsModule : public mcu::ISetupModule {
public:
    const char* name() const override { return "Sensors"; }

    mcu::Result<void> setup(mcu::ServiceContext& ctx) override {
        // Initialize IMU
        if (ctx.imu) {
            bool imuOk = ctx.imu->begin(Pins::I2C_SDA, Pins::I2C_SCL, 0x68);
            Serial.printf("[SENSORS] IMU init: %s\n", imuOk ? "OK" : "FAILED");
        }

        // Initialize LiDAR
        if (ctx.lidar) {
            bool lidarOk = ctx.lidar->begin(Pins::I2C_SDA, Pins::I2C_SCL);
            Serial.printf("[SENSORS] LiDAR init: %s\n", lidarOk ? "OK" : "FAILED");
        }

        Serial.println("[SENSORS] Sensor initialization complete");

        return mcu::Result<void>::ok();
    }
};

SetupSensorsModule g_setupSensors;

} // anonymous namespace

mcu::ISetupModule* getSetupSensorsModule() {
    return &g_setupSensors;
}
