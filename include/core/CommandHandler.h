#pragma once

#include <string>
#include <ArduinoJson.h>

#include "EventBus.h"
#include "Messages.h"
#include "SafetyManager.h"
#include "ModeManager.h"
#include "MotionController.h"
#include "managers/GpioManager.h"
#include "managers/PwmManager.h"
#include "managers/ServoManager.h"
#include "managers/StepperManager.h"

// Bring just this type into scope for this header
using ArduinoJson::JsonVariantConst;

class CommandHandler {
public:
    CommandHandler(EventBus&         bus,
                   ModeManager&      mode,
                   MotionController& motion,
                   SafetyManager&    safety,
                   GpioManager&      gpio,
                   PwmManager&       pwm,
                   ServoManager&     servo,
                   StepperManager&   stepper);

    // Call this once in setup() to attach to the EventBus
    void setup();

    // Raw JSON string from MessageRouter
    void onJsonCommand(const std::string& jsonStr);

    // === Core robot behavior handlers ===
    void handleSetMode(JsonVariantConst payload);
    void handleSetVel(JsonVariantConst payload);
    void handleStop();
    void handleEstop();
    void handleClearEstop();
    void handleLedOn();
    void handleLedOff();

    // === GPIO handlers ===
    void handleGpioWrite(JsonVariantConst payload);
    void handleGpioRead(JsonVariantConst payload);
    void handleGpioToggle(JsonVariantConst payload);
    void handleGpioRegisterChannel(JsonVariantConst payload);

    // === PWM / Servo / Stepper handlers ===
    void handlePwmSet(JsonVariantConst payload);

    void handleServoAttach(JsonVariantConst payload);
    void handleServoDetach(JsonVariantConst payload);
    void handleServoSetAngle(JsonVariantConst payload);

    void handleStepperMoveRel(JsonVariantConst payload);
    void handleStepperStop(JsonVariantConst payload);
    
    // Log handling 
    void handleSetLogLevel(JsonVariantConst payload);

private:
    EventBus&         bus_;
    ModeManager&      mode_;
    MotionController& motion_;
    SafetyManager&    safety_;

    GpioManager&      gpio_;
    PwmManager&       pwm_;
    ServoManager&     servo_;
    StepperManager&   stepper_;

    // === Static trampoline for EventBus ===
    static CommandHandler* s_instance;

    // Function-pointer handler that EventBus will call:
    static void handleEventStatic(const Event& evt);

    // Instance method that does the actual work:
    void handleEvent(const Event& evt);
    

};
