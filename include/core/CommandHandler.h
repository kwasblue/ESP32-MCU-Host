// include/core/CommandHandler.h
// Handles JSON commands from host and dispatches to appropriate handlers

#pragma once

#include <string>
#include <ArduinoJson.h>
#include "core/EventBus.h"
#include "config/CommandDefs.h"
#include "managers/GpioManager.h"
#include "managers/PwmManager.h"
#include "managers/ServoManager.h"
#include "managers/StepperManager.h"
#include "managers/UltrasonicManager.h"
#include "managers/EncoderManager.h"
#include "managers/DcMotorManager.h"

// Forward declarations
class ModeManager;
class MotionController;
class TelemetryModule;
class ControlModule;

class CommandHandler {
public:
    CommandHandler(EventBus&          bus,
                   ModeManager&       mode,
                   MotionController&  motion,
                   GpioManager&       gpio,
                   PwmManager&        pwm,
                   ServoManager&      servo,
                   StepperManager&    stepper,
                   TelemetryModule&   telemetry,
                   UltrasonicManager& ultrasonic,
                   EncoderManager&    encoder,
                   DcMotorManager&    dc);

    void setup();
    void onJsonCommand(const std::string& jsonStr);

    // Wire control module after construction
    void setControlModule(ControlModule* cm) { controlModule_ = cm; }

private:
    // Event handling
    static void handleEventStatic(const Event& evt);
    void handleEvent(const Event& evt);
    static CommandHandler* s_instance;

    // ACK helpers
    void sendAck(const char* cmd, bool ok, JsonDocument& resp);
    void sendError(const char* cmd, const char* error);
    void publishJson(std::string&& out);
    void publishJsonCopy(const std::string& out);

    // ACK cache for retry replay
    static constexpr int kAckCacheSize = 8;
    struct AckCacheEntry {
        bool valid = false;
        uint32_t seq = 0;
        CmdType cmdType = CmdType::UNKNOWN;
        std::string ackJson;
    };
    AckCacheEntry ackCache_[kAckCacheSize];
    int ackCacheWrite_ = 0;

    bool tryReplayAck(CmdType cmdType, uint32_t seq);
    void storeAck(CmdType cmdType, uint32_t seq, const std::string& ackJson);

    // Current command context
    uint32_t currentSeq_ = 0;
    CmdType currentCmdType_ = CmdType::UNKNOWN;

    // State guards
    bool requireIdle(const char* cmdName);
    bool requireArmed(const char* cmdName);

    // ----- Command Handlers -----

    // Safety / State Machine
    void handleHeartbeat();
    void handleArm();
    void handleDisarm();
    void handleActivate();
    void handleDeactivate();
    void handleEstop();
    void handleClearEstop();
    void handleStop();

    // Legacy
    void handleSetMode(JsonVariantConst payload);

    // Motion
    void handleSetVel(JsonVariantConst payload);

    // LED
    void handleLedOn();
    void handleLedOff();

    // GPIO
    void handleGpioWrite(JsonVariantConst payload);
    void handleGpioRead(JsonVariantConst payload);
    void handleGpioToggle(JsonVariantConst payload);
    void handleGpioRegisterChannel(JsonVariantConst payload);

    // PWM
    void handlePwmSet(JsonVariantConst payload);

    // Servo
    void handleServoAttach(JsonVariantConst payload);
    void handleServoDetach(JsonVariantConst payload);
    void handleServoSetAngle(JsonVariantConst payload);

    // Stepper
    void handleStepperEnable(JsonVariantConst payload);
    void handleStepperMoveRel(JsonVariantConst payload);
    void handleStepperStop(JsonVariantConst payload);

    // Ultrasonic
    void handleUltrasonicAttach(JsonVariantConst payload);
    void handleUltrasonicRead(JsonVariantConst payload);

    // Encoder
    void handleEncoderAttach(JsonVariantConst payload);
    void handleEncoderRead(JsonVariantConst payload);
    void handleEncoderReset(JsonVariantConst payload);

    // DC Motor
    void handleDcSetSpeed(JsonVariantConst payload);
    void handleDcStop(JsonVariantConst payload);
    void handleDcVelPidEnable(JsonVariantConst payload);
    void handleDcSetVelTarget(JsonVariantConst payload);
    void handleDcSetVelGains(JsonVariantConst payload);

    // Telemetry
    void handleTelemSetInterval(JsonVariantConst payload);

    // Logging
    void handleSetLogLevel(JsonVariantConst payload);

    // Loop Rates
    void handleGetRates(JsonVariantConst payload);
    void handleCtrlSetRate(JsonVariantConst payload);
    void handleSafetySetRate(JsonVariantConst payload);
    void handleTelemSetRate(JsonVariantConst payload);

    // Control Kernel - Slots
    void handleCtrlSlotConfig(JsonVariantConst payload);
    void handleCtrlSlotEnable(JsonVariantConst payload);
    void handleCtrlSlotReset(JsonVariantConst payload);
    void handleCtrlSlotSetParam(JsonVariantConst payload);
    void handleCtrlSlotStatus(JsonVariantConst payload);

    // Control Kernel - Signals
    void handleCtrlSignalDefine(JsonVariantConst payload);
    void handleCtrlSignalSet(JsonVariantConst payload);
    void handleCtrlSignalGet(JsonVariantConst payload);
    void handleCtrlSlotSetParamArray(JsonVariantConst payload);
    void handleCtrlSignalsList(JsonVariantConst payload);
    void handleCtrlSignalDelete(JsonVariantConst payload);
    void handleCtrlSignalsClear(JsonVariantConst payload);

    // Observer
    void handleObserverConfig(JsonVariantConst payload);
    void handleObserverEnable(JsonVariantConst payload);
    void handleObserverReset(JsonVariantConst payload);
    void handleObserverSetParam(JsonVariantConst payload);
    void handleObserverSetParamArray(JsonVariantConst payload);
    void handleObserverStatus(JsonVariantConst payload);


    // ----- Members -----
    EventBus&          bus_;
    ModeManager&       mode_;
    MotionController&  motion_;
    GpioManager&       gpio_;
    PwmManager&        pwm_;
    ServoManager&      servo_;
    StepperManager&    stepper_;
    TelemetryModule&   telemetry_;
    UltrasonicManager& ultrasonic_;
    EncoderManager&    encoder_;
    DcMotorManager&    dc_;
    ControlModule*     controlModule_ = nullptr;
};

// Helper to clamp Hz and return whether it was in range
inline bool clampHz(uint16_t& hz, uint16_t min_hz, uint16_t max_hz) {
    if (hz < min_hz) { hz = min_hz; return false; }
    if (hz > max_hz) { hz = max_hz; return false; }
    return true;
}