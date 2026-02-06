#include "setup/ISetupModule.h"
#include "core/ServiceContext.h"

#include <Arduino.h>
#include "config/PinConfig.h"
#include "config/GpioChannelDefs.h"
#include "hw/GpioManager.h"
#include "hw/PwmManager.h"
#include "motor/DcMotorManager.h"
#include "motor/StepperManager.h"

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

        // Register stepper motor
        if (ctx.stepper) {
            ctx.stepper->registerStepper(
                0,
                Pins::STEPPER0_STEP,
                Pins::STEPPER0_DIR,
                Pins::STEPPER0_EN,
                false
            );
            ctx.stepper->dumpAllStepperMappings();
        }

        // Attach DC motor
        if (ctx.dcMotor) {
            bool dcOk = ctx.dcMotor->attach(
                0,
                Pins::MOTOR_LEFT_IN1,
                Pins::MOTOR_LEFT_IN2,
                Pins::MOTOR_LEFT_PWM,
                0,      // LEDC channel
                15000,  // PWM frequency
                12      // Resolution bits
            );
            (void)dcOk;

            ctx.dcMotor->dumpAllMotorMappings();
        }

        Serial.println("[MOTORS] GPIO and motors configured");

        return mcu::Result<void>::ok();
    }
};

SetupMotorsModule g_setupMotors;

} // anonymous namespace

mcu::ISetupModule* getSetupMotorsModule() {
    return &g_setupMotors;
}
