#include "setup/ISetupModule.h"
#include "core/ServiceContext.h"

#include <Arduino.h>
#include "config/PinConfig.h"
#include "config/FeatureFlags.h"
#include "config/GpioChannelDefs.h"
#include "hw/GpioManager.h"
#include "hw/PwmManager.h"
#include "motor/DcMotorManager.h"
#include "motor/StepperManager.h"
#include "motor/ActuatorConfig.h"

// Include all self-registering actuators (triggers static registration)
#include "motor/AllActuators.h"

namespace {

class SetupMotorsModule : public mcu::ISetupModule {
public:
    const char* name() const override { return "Motors"; }

    mcu::Result<void> setup(mcu::ServiceContext& ctx) override {
        if (!ctx.gpio) {
            return mcu::Result<void>::err(mcu::ErrorCode::NotInitialized);
        }

        // Setup status LED
        pinMode(Pins::LED_STATUS, OUTPUT);
        digitalWrite(Pins::LED_STATUS, LOW);

        // Register GPIO channels from auto-generated definitions
        for (size_t i = 0; i < GPIO_CHANNEL_COUNT; ++i) {
            const auto& def = GPIO_CHANNEL_DEFS[i];
            ctx.gpio->registerChannel(def.channel, def.pin, def.mode);
        }

        // Build actuator capability mask from feature flags
        uint32_t actuatorCaps = 0;
#if HAS_DC_MOTOR
        actuatorCaps |= mcu::ActuatorCap::DC_MOTOR;
#endif
#if HAS_SERVO
        actuatorCaps |= mcu::ActuatorCap::SERVO;
#endif
#if HAS_STEPPER
        actuatorCaps |= mcu::ActuatorCap::STEPPER;
#endif
#if HAS_ENCODER
        actuatorCaps |= mcu::ActuatorCap::ENCODER;
#endif

        // Initialize self-registered actuators
        if (ctx.actuatorRegistry) {
            ctx.actuatorRegistry->setAvailableCaps(actuatorCaps);
            ctx.actuatorRegistry->initAll(ctx);
            ctx.actuatorRegistry->setupAll();
            Serial.printf("[MOTORS] Initialized %zu self-registered actuators\n",
                          ctx.actuatorRegistry->count());
        }

        // Legacy actuators (until fully migrated)
        // Auto-configure steppers from Pins::
        if (ctx.stepper) {
            mcu::autoConfigureSteppers(*ctx.stepper);
            ctx.stepper->dumpAllStepperMappings();
        }

        // Legacy DC motors (can be removed once DcMotorActuator is fully tested)
        if (ctx.dcMotor) {
            int dcCount = mcu::autoConfigureDcMotors(*ctx.dcMotor);
            Serial.printf("[MOTORS] Legacy: Auto-configured %d DC motors\n", dcCount);
            ctx.dcMotor->dumpAllMotorMappings();
        }

        // Auto-configure encoders from Pins::
        if (ctx.encoder) {
            mcu::autoConfigureEncoders(*ctx.encoder);
        }

        Serial.println("[MOTORS] GPIO and actuators configured");

        return mcu::Result<void>::ok();
    }
};

SetupMotorsModule g_setupMotors;

} // anonymous namespace

mcu::ISetupModule* getSetupMotorsModule() {
    return &g_setupMotors;
}
