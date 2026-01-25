// include/core/CommandHandler.h

#pragma once

#include <string>
#include <ArduinoJson.h>

#include "EventBus.h"
#include "Messages.h"
#include "ModeManager.h"
#include "MotionController.h"
#include "managers/GpioManager.h"
#include "managers/PwmManager.h"
#include "managers/ServoManager.h"
#include "managers/StepperManager.h"
#include "managers/UltrasonicManager.h"
#include "managers/EncoderManager.h"
#include "managers/DcMotorManager.h"
#include "modules/TelemetryModule.h"

using ArduinoJson::JsonVariantConst;
using ArduinoJson::JsonDocument;

class CommandHandler {
public:
    // SafetyManager removed - now part of ModeManager
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

private:
    // ----- Event handling -----
    static CommandHandler* s_instance;
    static void handleEventStatic(const Event& evt);
    void handleEvent(const Event& evt);
    void onJsonCommand(const std::string& jsonStr);

    // ----- ACK helpers -----
        // ----- Sequence / retry protection -----
    static constexpr int kAckCacheSize = 16;

    struct AckCacheEntry {
        bool        valid = false;
        uint32_t    seq   = 0;
        CmdType     cmdType = CmdType::UNKNOWN;
        std::string ackJson;
    };

    AckCacheEntry ackCache_[kAckCacheSize];
    int ackCacheWrite_ = 0;

    uint32_t currentSeq_ = 0;
    CmdType  currentCmdType_ = CmdType::UNKNOWN;

    bool tryReplayAck(CmdType cmdType, uint32_t seq);
    void storeAck(CmdType cmdType, uint32_t seq, const std::string& ackJson);
    void publishJson(std::string&& out);
    void publishJsonCopy(const std::string& out);

    void sendAck(const char* cmd, bool ok, JsonDocument& resp);
    void sendError(const char* cmd, const char* error);

    // ----- Safety / State Machine -----
    void handleHeartbeat();
    void handleArm();
    void handleDisarm();
    void handleActivate();
    void handleDeactivate();
    void handleEstop();
    void handleClearEstop();
    void handleStop();

    // ----- Legacy (consider deprecating) -----
    void handleSetMode(JsonVariantConst payload);

    // ----- Motion -----
    void handleSetVel(JsonVariantConst payload);

    // ----- LED -----
    void handleLedOn();
    void handleLedOff();

    // ----- GPIO -----
    void handleGpioWrite(JsonVariantConst payload);
    void handleGpioRead(JsonVariantConst payload);
    void handleGpioToggle(JsonVariantConst payload);
    void handleGpioRegisterChannel(JsonVariantConst payload);

    // ----- PWM -----
    void handlePwmSet(JsonVariantConst payload);

    // ----- Servo -----
    void handleServoAttach(JsonVariantConst payload);
    void handleServoDetach(JsonVariantConst payload);
    void handleServoSetAngle(JsonVariantConst payload);

    // ----- Stepper -----
    void handleStepperEnable(JsonVariantConst payload);
    void handleStepperMoveRel(JsonVariantConst payload);
    void handleStepperStop(JsonVariantConst payload);

    // ----- Ultrasonic -----
    void handleUltrasonicAttach(JsonVariantConst payload);
    void handleUltrasonicRead(JsonVariantConst payload);

    // ----- Encoder -----
    void handleEncoderAttach(JsonVariantConst payload);
    void handleEncoderRead(JsonVariantConst payload);
    void handleEncoderReset(JsonVariantConst payload);

    // ----- DC Motor -----
    void handleDcSetSpeed(JsonVariantConst payload);
    void handleDcStop(JsonVariantConst payload);
    void handleDcVelPidEnable(JsonVariantConst payload);
    void handleDcSetVelTarget(JsonVariantConst payload);
    void handleDcSetVelGains(JsonVariantConst payload);

    // ----- Telemetry -----
    void handleTelemSetInterval(JsonVariantConst payload);

    // ----- Logging -----
    void handleSetLogLevel(JsonVariantConst payload);

    // ----- Member references -----
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
};

