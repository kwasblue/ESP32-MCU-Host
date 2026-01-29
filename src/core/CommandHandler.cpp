// src/core/CommandHandler.cpp

#include "core/CommandHandler.h"
#include "core/Messages.h"
#include "core/ModeManager.h"
#include "core/MotionController.h"
#include "config/PinConfig.h"
#include "core/Event.h"
#include "modules/LoggingModule.h"
#include "modules/TelemetryModule.h"
#include "managers/UltrasonicManager.h"
#include "managers/StepperManager.h"
#include "managers/EncoderManager.h"
#include "managers/DcMotorManager.h"
#include "core/Debug.h"
#include <Arduino.h>
#include "core/LoopRates.h"
#include "core/ModeManager.h"
#include "modules/ControlModule.h"
#include "core/SignalBus.h"


using namespace ArduinoJson;

CommandHandler* CommandHandler::s_instance = nullptr;

CommandHandler::CommandHandler(EventBus&          bus,
                               ModeManager&       mode,
                               MotionController&  motion,
                               GpioManager&       gpio,
                               PwmManager&        pwm,
                               ServoManager&      servo,
                               StepperManager&    stepper,
                               TelemetryModule&   telemetry,
                               UltrasonicManager& ultrasonic,
                               EncoderManager&    encoder,
                               DcMotorManager&    dc)
    : bus_(bus)
    , mode_(mode)
    , motion_(motion)
    , gpio_(gpio)
    , pwm_(pwm)
    , servo_(servo)
    , stepper_(stepper)
    , telemetry_(telemetry)
    , ultrasonic_(ultrasonic)
    , encoder_(encoder)
    , dc_(dc)
{
    s_instance = this;
}

// -----------------------------------------------------------------------------
// Setup / Event subscription
// -----------------------------------------------------------------------------
void CommandHandler::setup() {
    DBG_PRINTLN("[CMD] CommandHandler::setup() subscribing");
    bus_.subscribe(&CommandHandler::handleEventStatic);
    DBG_PRINTLN("[CMD] CommandHandler::setup() done");
}

void CommandHandler::handleEventStatic(const Event& evt) {
    if (s_instance) {
        s_instance->handleEvent(evt);
    }
}

void CommandHandler::handleEvent(const Event& evt) {
    if (evt.type != EventType::JSON_MESSAGE_RX) {
        return;
    }
    mode_.onHostHeartbeat(millis());
    onJsonCommand(evt.payload.json);
}

// -----------------------------------------------------------------------------
// JSON Command Dispatch
// -----------------------------------------------------------------------------
void CommandHandler::onJsonCommand(const std::string& jsonStr) {
    DBG_PRINT("[CMD] raw JSON: ");
    DBG_PRINTLN(jsonStr.c_str());

    JsonMessage msg;
    if (!parseJsonToMessage(jsonStr, msg)) {
        DBG_PRINT("[CMD] Failed to parse JSON: ");
        DBG_PRINTLN(jsonStr.c_str());
        return;
    }

    DBG_PRINTF("[CMD] kind=%d typeStr=%s cmdType=%d\n",
               (int)msg.kind, msg.typeStr.c_str(), (int)msg.cmdType);

    // If not a command, we're done
    if (msg.kind != MsgKind::CMD) {
        DBG_PRINT("[CMD] Ignoring non-command: ");
        DBG_PRINTLN(msg.typeStr.c_str());
        return;
    }

    // Set current context for ACK/Error helpers
    currentSeq_ = msg.seq;
    currentCmdType_ = msg.cmdType;

    // Check for duplicate command replay
    if (tryReplayAck(msg.cmdType, msg.seq)) {
        return;
    }

    JsonVariantConst payload = msg.payload.as<JsonVariantConst>();

    switch (msg.cmdType) {
        // ----- Safety / State Machine -----
        case CmdType::HEARTBEAT:              handleHeartbeat();                 break;
        case CmdType::ARM:                    handleArm();                       break;
        case CmdType::DISARM:                 handleDisarm();                    break;
        case CmdType::ACTIVATE:               handleActivate();                  break;
        case CmdType::DEACTIVATE:             handleDeactivate();                break;
        case CmdType::ESTOP:                  handleEstop();                     break;
        case CmdType::CLEAR_ESTOP:            handleClearEstop();                break;
        case CmdType::STOP:                   handleStop();                      break;

        // ----- Legacy (consider deprecating) -----
        case CmdType::SET_MODE:               handleSetMode(payload);            break;

        // ----- Motion -----
        case CmdType::SET_VEL:                handleSetVel(payload);             break;

        // ----- LED -----
        case CmdType::LED_ON:                 handleLedOn();                     break;
        case CmdType::LED_OFF:                handleLedOff();                    break;

        // ----- GPIO -----
        case CmdType::GPIO_WRITE:             handleGpioWrite(payload);          break;
        case CmdType::GPIO_READ:              handleGpioRead(payload);           break;
        case CmdType::GPIO_TOGGLE:            handleGpioToggle(payload);         break;
        case CmdType::GPIO_REGISTER_CHANNEL:  handleGpioRegisterChannel(payload);break;

        // ----- PWM -----
        case CmdType::PWM_SET:                handlePwmSet(payload);             break;

        // ----- Servo -----
        case CmdType::SERVO_ATTACH:           handleServoAttach(payload);        break;
        case CmdType::SERVO_DETACH:           handleServoDetach(payload);        break;
        case CmdType::SERVO_SET_ANGLE:        handleServoSetAngle(payload);      break;

        // ----- Stepper -----
        case CmdType::STEPPER_ENABLE:         handleStepperEnable(payload);      break;
        case CmdType::STEPPER_MOVE_REL:       handleStepperMoveRel(payload);     break;
        case CmdType::STEPPER_STOP:           handleStepperStop(payload);        break;

        // ----- Ultrasonic -----
        case CmdType::ULTRASONIC_ATTACH:      handleUltrasonicAttach(payload);   break;
        case CmdType::ULTRASONIC_READ:        handleUltrasonicRead(payload);     break;

        // ----- Encoder -----
        case CmdType::ENCODER_ATTACH:         handleEncoderAttach(payload);      break;
        case CmdType::ENCODER_READ:           handleEncoderRead(payload);        break;
        case CmdType::ENCODER_RESET:          handleEncoderReset(payload);       break;

        // ----- DC Motor -----
        case CmdType::DC_SET_SPEED:           handleDcSetSpeed(payload);         break;
        case CmdType::DC_STOP:                handleDcStop(payload);             break;
        case CmdType::DC_VEL_PID_ENABLE:      handleDcVelPidEnable(payload);     break;
        case CmdType::DC_SET_VEL_TARGET:      handleDcSetVelTarget(payload);     break;
        case CmdType::DC_SET_VEL_GAINS:       handleDcSetVelGains(payload);      break;

        // ----- Telemetry -----
        case CmdType::TELEM_SET_INTERVAL:     handleTelemSetInterval(payload);   break;

        // ----- Loop Rates -----
        case CmdType::GET_RATES:              handleGetRates(payload);           break;
        case CmdType::CTRL_SET_RATE:          handleCtrlSetRate(payload);        break;
        case CmdType::SAFETY_SET_RATE:        handleSafetySetRate(payload);      break;
        case CmdType::TELEM_SET_RATE:         handleTelemSetRate(payload);       break;

        // ----- Control Kernel -----
        case CmdType::CTRL_SIGNAL_DEFINE:     handleCtrlSignalDefine(payload);   break;
        case CmdType::CTRL_SLOT_CONFIG:       handleCtrlSlotConfig(payload);     break;
        case CmdType::CTRL_SLOT_ENABLE:       handleCtrlSlotEnable(payload);     break;
        case CmdType::CTRL_SLOT_RESET:        handleCtrlSlotReset(payload);      break;
        case CmdType::CTRL_SLOT_SET_PARAM:    handleCtrlSlotSetParam(payload);   break;
        case CmdType::CTRL_SLOT_STATUS:       handleCtrlSlotStatus(payload);     break;
        case CmdType::CTRL_SIGNAL_SET:        handleCtrlSignalSet(payload);      break;
        case CmdType::CTRL_SIGNAL_GET:        handleCtrlSignalGet(payload);      break;
        case CmdType::CTRL_SLOT_SET_PARAM_ARRAY: handleCtrlSlotSetParamArray(payload); break;
        
        case CmdType::CTRL_SIGNALS_LIST:      handleCtrlSignalsList(payload);    break;

        // ----- Logging -----
        case CmdType::SET_LOG_LEVEL:          handleSetLogLevel(payload);        break;

        default:
            DBG_PRINTF("[CMD] Unknown cmdType: %s\n", msg.typeStr.c_str());
            sendError("UNKNOWN_CMD", "unknown_command");
            break;
    }
}

// -----------------------------------------------------------------------------
// ACK Helpers
// -----------------------------------------------------------------------------
bool CommandHandler::tryReplayAck(CmdType cmdType, uint32_t seq) {
    if (seq == 0) return false;

    for (int i = 0; i < kAckCacheSize; ++i) {
        auto& e = ackCache_[i];
        if (e.valid && e.seq == seq && e.cmdType == cmdType) {
            DBG_PRINTF("[CMD] Duplicate cmd detected. Replaying ACK cmdType=%d seq=%lu\n",
                       (int)cmdType, (unsigned long)seq);
            publishJsonCopy(e.ackJson);
            return true;
        }
    }
    return false;
}

void CommandHandler::storeAck(CmdType cmdType, uint32_t seq, const std::string& ackJson) {
    if (seq == 0) return;

    auto& e = ackCache_[ackCacheWrite_];
    e.valid = true;
    e.seq = seq;
    e.cmdType = cmdType;
    e.ackJson = ackJson;
    ackCacheWrite_ = (ackCacheWrite_ + 1) % kAckCacheSize;
}

void CommandHandler::publishJson(std::string&& out) {
    Event evt;
    evt.type         = EventType::JSON_MESSAGE_TX;
    evt.payload.json = std::move(out);
    bus_.publish(evt);
}

void CommandHandler::publishJsonCopy(const std::string& out) {
    std::string copy = out;
    publishJson(std::move(copy));
}

void CommandHandler::sendAck(const char* cmd, bool ok, JsonDocument& resp) {
    resp["src"] = "mcu";
    resp["cmd"] = cmd;
    resp["ok"]  = ok;
    resp["seq"] = currentSeq_;

    std::string out;
    serializeJson(resp, out);
    storeAck(currentCmdType_, currentSeq_, out);
    publishJson(std::move(out));
}

void CommandHandler::sendError(const char* cmd, const char* error) {
    JsonDocument resp;
    resp["src"]   = "mcu";
    resp["cmd"]   = cmd;
    resp["ok"]    = false;
    resp["error"] = error;
    resp["seq"]   = currentSeq_;

    std::string out;
    serializeJson(resp, out);
    storeAck(currentCmdType_, currentSeq_, out);
    publishJson(std::move(out));
}

// -----------------------------------------------------------------------------
// State Guards
// -----------------------------------------------------------------------------
bool CommandHandler::requireIdle(const char* cmdName) {
    if (mode_.mode() == RobotMode::IDLE) {
        return true;
    }
    sendError(cmdName, "not_idle");
    return false;
}

bool CommandHandler::requireArmed(const char* cmdName) {
    if (mode_.canMove()) {
        return true;
    }
    sendError(cmdName, "not_armed");
    return false;
}

// -----------------------------------------------------------------------------
// Safety / State Machine Commands
// -----------------------------------------------------------------------------
void CommandHandler::handleHeartbeat() {
    DBG_PRINTLN("[CMD] HEARTBEAT");
    mode_.onHostHeartbeat(millis());

    JsonDocument resp;
    resp["state"] = robotModeToString(mode_.mode());
    sendAck("CMD_HEARTBEAT", true, resp);
}

void CommandHandler::handleArm() {
    DBG_PRINTLN("[CMD] ARM");
    mode_.arm();

    JsonDocument resp;
    resp["state"] = robotModeToString(mode_.mode());
    bool ok = (mode_.mode() == RobotMode::ARMED);
    if (!ok) {
        resp["error"] = "invalid_transition";
    }
    sendAck("CMD_ARM", ok, resp);
}

void CommandHandler::handleDisarm() {
    DBG_PRINTLN("[CMD] DISARM");
    mode_.disarm();

    JsonDocument resp;
    resp["state"] = robotModeToString(mode_.mode());
    bool ok = (mode_.mode() == RobotMode::IDLE);
    if (!ok) {
        resp["error"] = "invalid_transition";
    }
    sendAck("CMD_DISARM", ok, resp);
}

void CommandHandler::handleActivate() {
    DBG_PRINTLN("[CMD] ACTIVATE");
    mode_.activate();

    JsonDocument resp;
    resp["state"] = robotModeToString(mode_.mode());
    bool ok = (mode_.mode() == RobotMode::ACTIVE);
    if (!ok) {
        resp["error"] = "invalid_transition";
    }
    sendAck("CMD_ACTIVATE", ok, resp);
}

void CommandHandler::handleDeactivate() {
    DBG_PRINTLN("[CMD] DEACTIVATE");
    mode_.deactivate();

    JsonDocument resp;
    resp["state"] = robotModeToString(mode_.mode());
    sendAck("CMD_DEACTIVATE", true, resp);
}

void CommandHandler::handleEstop() {
    DBG_PRINTLN("[CMD] ESTOP");
    mode_.estop();

    JsonDocument resp;
    resp["state"] = robotModeToString(mode_.mode());
    sendAck("CMD_ESTOP", true, resp);
}

void CommandHandler::handleClearEstop() {
    DBG_PRINTLN("[CMD] CLEAR_ESTOP");
    bool cleared = mode_.clearEstop();

    JsonDocument resp;
    resp["state"] = robotModeToString(mode_.mode());
    if (!cleared) {
        resp["error"] = "cannot_clear";
    }
    sendAck("CMD_CLEAR_ESTOP", cleared, resp);
}

void CommandHandler::handleStop() {
    DBG_PRINTLN("[CMD] STOP");
    motion_.stop();

    JsonDocument resp;
    sendAck("CMD_STOP", true, resp);
}

// -----------------------------------------------------------------------------
// Legacy SET_MODE (consider deprecating)
// -----------------------------------------------------------------------------
void CommandHandler::handleSetMode(JsonVariantConst payload) {
    const char* modeStr = payload["mode"] | "IDLE";
    DBG_PRINTF("[CMD] SET_MODE mode=%s\n", modeStr);

    bool ok = true;
    const char* error = nullptr;

    if (mode_.isEstopped()) {
        ok = false;
        error = "in_estop";
    } else if (strcmp(modeStr, "IDLE") == 0) {
        mode_.disarm();
    } else if (strcmp(modeStr, "ARMED") == 0) {
        mode_.arm();
    } else if (strcmp(modeStr, "ACTIVE") == 0) {
        mode_.arm();
        mode_.activate();
    } else {
        ok = false;
        error = "unsupported_mode";
    }

    JsonDocument resp;
    resp["mode"]  = modeStr;
    resp["state"] = robotModeToString(mode_.mode());
    if (!ok && error) {
        resp["error"] = error;
    }
    sendAck("CMD_SET_MODE", ok, resp);
}

// -----------------------------------------------------------------------------
// Motion
// -----------------------------------------------------------------------------
void CommandHandler::handleSetVel(JsonVariantConst payload) {
    const uint32_t now_ms = millis();

    float vx    = payload["vx"]    | 0.0f;
    float omega = payload["omega"] | 0.0f;

    float safe_vx = 0.0f, safe_omega = 0.0f;
    if (!mode_.validateVelocity(vx, omega, safe_vx, safe_omega)) {
        DBG_PRINTF("[CMD] SET_VEL invalid: vx=%f omega=%f\n", vx, omega);
        sendError("CMD_SET_VEL", "invalid_velocity");
        return;
    }

    // Always allow "STOP" even if not armed
    const bool is_stop_cmd = (fabsf(safe_vx) < 1e-6f) && (fabsf(safe_omega) < 1e-6f);
    if (is_stop_cmd) {
        mode_.onMotionCommand(now_ms);
        motion_.stop();

        JsonDocument resp;
        resp["vx"]    = safe_vx;
        resp["omega"] = safe_omega;
        resp["state"] = robotModeToString(mode_.mode());
        sendAck("CMD_SET_VEL", true, resp);
        return;
    }

    // Gate non-zero motion
    if (!mode_.canMove()) {
        DBG_PRINTF("[CMD] SET_VEL rejected: mode=%s\n", robotModeToString(mode_.mode()));
        sendError("CMD_SET_VEL", "not_armed");
        return;
    }

    mode_.onMotionCommand(now_ms);
    motion_.setVelocity(safe_vx, safe_omega);

    JsonDocument resp;
    resp["vx"]    = safe_vx;
    resp["omega"] = safe_omega;
    resp["state"] = robotModeToString(mode_.mode());
    sendAck("CMD_SET_VEL", true, resp);
}

// -----------------------------------------------------------------------------
// LED
// -----------------------------------------------------------------------------
void CommandHandler::handleLedOn() {
    DBG_PRINTLN("[CMD] LED_ON");
    digitalWrite(Pins::LED_STATUS, HIGH);

    JsonDocument resp;
    resp["pin"] = Pins::LED_STATUS;
    resp["on"]  = true;
    sendAck("CMD_LED_ON", true, resp);
}

void CommandHandler::handleLedOff() {
    DBG_PRINTLN("[CMD] LED_OFF");
    digitalWrite(Pins::LED_STATUS, LOW);

    JsonDocument resp;
    resp["pin"] = Pins::LED_STATUS;
    resp["on"]  = false;
    sendAck("CMD_LED_OFF", true, resp);
}

// -----------------------------------------------------------------------------
// GPIO
// -----------------------------------------------------------------------------
void CommandHandler::handleGpioWrite(JsonVariantConst payload) {
    int ch  = payload["channel"] | -1;
    int val = payload["value"]   | 0;

    bool ok = gpio_.hasChannel(ch);
    if (ok) {
        gpio_.write(ch, val);
    }

    DBG_PRINTF("[CMD] GPIO_WRITE ch=%d val=%d ok=%d\n", ch, val, (int)ok);

    JsonDocument resp;
    resp["channel"] = ch;
    resp["value"]   = val;
    if (!ok) {
        resp["error"] = "invalid_channel";
    }
    sendAck("CMD_GPIO_WRITE", ok, resp);
}

void CommandHandler::handleGpioRead(JsonVariantConst payload) {
    int ch = payload["channel"] | -1;

    bool ok  = gpio_.hasChannel(ch);
    int  val = ok ? gpio_.read(ch) : -1;

    DBG_PRINTF("[CMD] GPIO_READ ch=%d val=%d ok=%d\n", ch, val, (int)ok);

    JsonDocument resp;
    resp["channel"] = ch;
    resp["value"]   = val;
    if (!ok) {
        resp["error"] = "invalid_channel";
    }
    sendAck("CMD_GPIO_READ", ok, resp);
}

void CommandHandler::handleGpioToggle(JsonVariantConst payload) {
    int ch = payload["channel"] | -1;

    bool ok = gpio_.hasChannel(ch);
    if (ok) {
        gpio_.toggle(ch);
    }

    DBG_PRINTF("[CMD] GPIO_TOGGLE ch=%d ok=%d\n", ch, (int)ok);

    JsonDocument resp;
    resp["channel"] = ch;
    if (!ok) {
        resp["error"] = "invalid_channel";
    }
    sendAck("CMD_GPIO_TOGGLE", ok, resp);
}

void CommandHandler::handleGpioRegisterChannel(JsonVariantConst payload) {
    int         ch      = payload["channel"] | -1;
    int         pin     = payload["pin"]     | -1;
    const char* modeStr = payload["mode"]    | "output";

    int mode = OUTPUT;
    if (strcmp(modeStr, "input") == 0) {
        mode = INPUT;
    } else if (strcmp(modeStr, "input_pullup") == 0) {
        mode = INPUT_PULLUP;
    }

    bool ok = (ch >= 0 && pin >= 0);
    if (ok) {
        gpio_.registerChannel(ch, pin, mode);
    }

    DBG_PRINTF("[CMD] GPIO_REGISTER ch=%d pin=%d mode=%s ok=%d\n",
               ch, pin, modeStr, (int)ok);

    JsonDocument resp;
    resp["channel"] = ch;
    resp["pin"]     = pin;
    resp["mode"]    = modeStr;
    if (!ok) {
        resp["error"] = "invalid_params";
    }
    sendAck("CMD_GPIO_REGISTER_CHANNEL", ok, resp);
}

// -----------------------------------------------------------------------------
// PWM
// -----------------------------------------------------------------------------
void CommandHandler::handlePwmSet(JsonVariantConst payload) {
    int   channel = payload["channel"] | 0;
    float duty    = payload["duty"]    | 0.0f;
    float freq    = payload["freq_hz"] | 0.0f;

    DBG_PRINTF("[CMD] PWM_SET ch=%d duty=%.3f freq=%.1f\n", channel, duty, freq);

    pwm_.set(channel, duty, freq);

    JsonDocument resp;
    resp["channel"] = channel;
    resp["duty"]    = duty;
    resp["freq_hz"] = freq;
    sendAck("CMD_PWM_SET", true, resp);
}

// -----------------------------------------------------------------------------
// Servo
// -----------------------------------------------------------------------------
void CommandHandler::handleServoAttach(JsonVariantConst payload) {
    int servoId = payload["servo_id"] | 0;
    int minUs   = payload["min_us"]   | 1000;
    int maxUs   = payload["max_us"]   | 2000;

    uint8_t pin = 0;
    bool ok = true;

    switch (servoId) {
        case 0: pin = Pins::SERVO1_SIG; break;
        default:
            ok = false;
            break;
    }

    if (ok) {
        DBG_PRINTF("[CMD] SERVO_ATTACH id=%d pin=%d min=%d max=%d\n",
                   servoId, pin, minUs, maxUs);
        servo_.attach(servoId, pin, minUs, maxUs);
    } else {
        DBG_PRINTF("[CMD] SERVO_ATTACH: unknown servoId=%d\n", servoId);
    }

    JsonDocument resp;
    resp["servo_id"] = servoId;
    resp["pin"]      = pin;
    resp["min_us"]   = minUs;
    resp["max_us"]   = maxUs;
    if (!ok) {
        resp["error"] = "unknown_servo_id";
    }
    sendAck("CMD_SERVO_ATTACH", ok, resp);
}

void CommandHandler::handleServoDetach(JsonVariantConst payload) {
    int servoId = payload["servo_id"] | 0;

    DBG_PRINTF("[CMD] SERVO_DETACH id=%d\n", servoId);
    servo_.detach(servoId);

    JsonDocument resp;
    resp["servo_id"] = servoId;
    sendAck("CMD_SERVO_DETACH", true, resp);
}

void CommandHandler::handleServoSetAngle(JsonVariantConst payload) {
    if (!mode_.canMove()) {
        sendError("CMD_SERVO_SET_ANGLE", "not_armed");
        return;
    }

    int   servoId = payload["servo_id"]    | 0;
    float angle   = payload["angle_deg"]   | 0.0f;
    int   durMs   = payload["duration_ms"] | 0;

    DBG_PRINTF("[CMD] SERVO_SET_ANGLE id=%d angle=%.1f dur=%d\n",
               servoId, angle, durMs);

    mode_.onMotionCommand(millis());

    if (durMs <= 0) {
        servo_.setAngle(servoId, angle);
    } else {
        motion_.setServoTarget(servoId, angle, durMs);
    }

    JsonDocument resp;
    resp["servo_id"]    = servoId;
    resp["angle_deg"]   = angle;
    resp["duration_ms"] = durMs;
    sendAck("CMD_SERVO_SET_ANGLE", true, resp);
}

// -----------------------------------------------------------------------------
// Stepper
// -----------------------------------------------------------------------------
void CommandHandler::handleStepperEnable(JsonVariantConst payload) {
    int  motorId = payload["motor_id"] | 0;
    bool enable  = payload["enable"]   | true;

    DBG_PRINTF("[CMD] STEPPER_ENABLE motor=%d enable=%d\n", motorId, (int)enable);
    stepper_.setEnabled(motorId, enable);

    JsonDocument resp;
    resp["motor_id"] = motorId;
    resp["enable"]   = enable;
    sendAck("CMD_STEPPER_ENABLE", true, resp);
}

void CommandHandler::handleStepperMoveRel(JsonVariantConst payload) {
    if (!mode_.canMove()) {
        sendError("CMD_STEPPER_MOVE_REL", "not_armed");
        return;
    }

    int   motorId = payload["motor_id"]      | 0;
    int   steps   = payload["steps"]         | 0;
    float speed   = payload["speed_steps_s"] | 1000.0f;

    DBG_PRINTF("[CMD] STEPPER_MOVE_REL motor=%d steps=%d speed=%.1f\n",
               motorId, steps, speed);

    mode_.onMotionCommand(millis());
    motion_.moveStepperRelative(motorId, steps, speed);

    JsonDocument resp;
    resp["motor_id"]      = motorId;
    resp["steps"]         = steps;
    resp["speed_steps_s"] = speed;
    sendAck("CMD_STEPPER_MOVE_REL", true, resp);
}

void CommandHandler::handleStepperStop(JsonVariantConst payload) {
    int motorId = payload["motor_id"] | 0;

    DBG_PRINTF("[CMD] STEPPER_STOP motor=%d\n", motorId);
    stepper_.stop(motorId);

    JsonDocument resp;
    resp["motor_id"] = motorId;
    sendAck("CMD_STEPPER_STOP", true, resp);
}

// -----------------------------------------------------------------------------
// Ultrasonic
// -----------------------------------------------------------------------------
void CommandHandler::handleUltrasonicAttach(JsonVariantConst payload) {
    int sensorId = payload["sensor_id"] | 0;

    uint8_t trigPin = 0;
    uint8_t echoPin = 0;
    bool ok = true;

    switch (sensorId) {
        case 0:
            trigPin = Pins::ULTRA0_TRIG;
            echoPin = Pins::ULTRA0_ECHO;
            break;
        default:
            ok = false;
            break;
    }

    if (ok) {
        DBG_PRINTF("[CMD] ULTRASONIC_ATTACH id=%d trig=%d echo=%d\n",
                   sensorId, trigPin, echoPin);
        ok = ultrasonic_.attach(sensorId, trigPin, echoPin);
    } else {
        DBG_PRINTF("[CMD] ULTRASONIC_ATTACH: unknown sensorId=%d\n", sensorId);
    }

    JsonDocument resp;
    resp["sensor_id"] = sensorId;
    resp["trig_pin"]  = trigPin;
    resp["echo_pin"]  = echoPin;
    if (!ok) {
        resp["error"] = "attach_failed";
    }
    sendAck("CMD_ULTRASONIC_ATTACH", ok, resp);
}

void CommandHandler::handleUltrasonicRead(JsonVariantConst payload) {
    int sensorId = payload["sensor_id"] | 0;

    float distCm = ultrasonic_.readDistanceCm(sensorId);
    bool ok = (distCm >= 0.0f);

    DBG_PRINTF("[CMD] ULTRASONIC_READ id=%d dist=%.2f ok=%d\n",
               sensorId, distCm, (int)ok);

    JsonDocument resp;
    resp["sensor_id"]   = sensorId;
    resp["distance_cm"] = ok ? distCm : -1.0f;
    if (!ok) {
        resp["error"] = "read_failed";
    }
    sendAck("CMD_ULTRASONIC_READ", ok, resp);
}

// -----------------------------------------------------------------------------
// Encoder
// -----------------------------------------------------------------------------
void CommandHandler::handleEncoderAttach(JsonVariantConst payload) {
    int encoderId = payload["encoder_id"] | 0;
    int pinA      = payload["pin_a"]      | Pins::ENC0_A;
    int pinB      = payload["pin_b"]      | Pins::ENC0_B;

    DBG_PRINTF("[CMD] ENCODER_ATTACH id=%d pinA=%d pinB=%d\n",
               encoderId, pinA, pinB);

    encoder_.attach(
        static_cast<uint8_t>(encoderId),
        static_cast<gpio_num_t>(pinA),
        static_cast<gpio_num_t>(pinB)
    );

    JsonDocument resp;
    resp["encoder_id"] = encoderId;
    resp["pin_a"]      = pinA;
    resp["pin_b"]      = pinB;
    sendAck("CMD_ENCODER_ATTACH", true, resp);
}

void CommandHandler::handleEncoderRead(JsonVariantConst payload) {
    int encoderId = payload["encoder_id"] | 0;

    int32_t ticks = encoder_.getCount(static_cast<uint8_t>(encoderId));

    DBG_PRINTF("[CMD] ENCODER_READ id=%d ticks=%ld\n", encoderId, (long)ticks);

    JsonDocument resp;
    resp["encoder_id"] = encoderId;
    resp["ticks"]      = ticks;
    sendAck("CMD_ENCODER_READ", true, resp);
}

void CommandHandler::handleEncoderReset(JsonVariantConst payload) {
    int encoderId = payload["encoder_id"] | 0;

    DBG_PRINTF("[CMD] ENCODER_RESET id=%d\n", encoderId);
    encoder_.reset(static_cast<uint8_t>(encoderId));

    JsonDocument resp;
    resp["encoder_id"] = encoderId;
    sendAck("CMD_ENCODER_RESET", true, resp);
}

// -----------------------------------------------------------------------------
// DC Motor
// -----------------------------------------------------------------------------
void CommandHandler::handleDcSetSpeed(JsonVariantConst payload) {
    if (!mode_.canMove()) {
        sendError("CMD_DC_SET_SPEED", "not_armed");
        return;
    }

    int   motorId = payload["motor_id"] | 0;
    float speed   = payload["speed"]    | 0.0f;

    DBG_PRINTF("[CMD] DC_SET_SPEED motor=%d speed=%.3f\n", motorId, speed);

    mode_.onMotionCommand(millis());
    bool ok = dc_.setSpeed(static_cast<uint8_t>(motorId), speed);

    JsonDocument resp;
    resp["motor_id"] = motorId;
    resp["speed"]    = speed;
    if (!ok) {
        resp["error"] = "set_speed_failed";
    }
    sendAck("CMD_DC_SET_SPEED", ok, resp);
}

void CommandHandler::handleDcStop(JsonVariantConst payload) {
    int motorId = payload["motor_id"] | 0;

    DBG_PRINTF("[CMD] DC_STOP motor=%d\n", motorId);
    dc_.stop(static_cast<uint8_t>(motorId));

    JsonDocument resp;
    resp["motor_id"] = motorId;
    sendAck("CMD_DC_STOP", true, resp);
}

void CommandHandler::handleDcVelPidEnable(JsonVariantConst payload) {
    int  motorId = payload["motor_id"] | 0;
    bool enable  = payload["enable"]   | true;

    DBG_PRINTF("[CMD] DC_VEL_PID_ENABLE motor=%d enable=%d\n", motorId, (int)enable);
    bool ok = dc_.enableVelocityPid(static_cast<uint8_t>(motorId), enable);

    JsonDocument resp;
    resp["motor_id"] = motorId;
    resp["enable"]   = enable;
    if (!ok) {
        resp["error"] = "enable_failed";
    }
    sendAck("CMD_DC_VEL_PID_ENABLE", ok, resp);
}

void CommandHandler::handleDcSetVelTarget(JsonVariantConst payload) {
    if (!mode_.canMove()) {
        sendError("CMD_DC_SET_VEL_TARGET", "not_armed");
        return;
    }

    int   motorId = payload["motor_id"] | 0;
    float omega   = payload["omega"]    | 0.0f;

    DBG_PRINTF("[CMD] DC_SET_VEL_TARGET motor=%d omega=%.3f\n", motorId, omega);

    mode_.onMotionCommand(millis());
    bool ok = dc_.setVelocityTarget(static_cast<uint8_t>(motorId), omega);

    JsonDocument resp;
    resp["motor_id"] = motorId;
    resp["omega"]    = omega;
    if (!ok) {
        resp["error"] = "set_target_failed";
    }
    sendAck("CMD_DC_SET_VEL_TARGET", ok, resp);
}

void CommandHandler::handleDcSetVelGains(JsonVariantConst payload) {
    int   motorId = payload["motor_id"] | 0;
    float kp      = payload["kp"]       | 0.0f;
    float ki      = payload["ki"]       | 0.0f;
    float kd      = payload["kd"]       | 0.0f;

    DBG_PRINTF("[CMD] DC_SET_VEL_GAINS motor=%d kp=%.4f ki=%.4f kd=%.4f\n",
               motorId, kp, ki, kd);

    bool ok = dc_.setVelocityGains(static_cast<uint8_t>(motorId), kp, ki, kd);

    JsonDocument resp;
    resp["motor_id"] = motorId;
    resp["kp"]       = kp;
    resp["ki"]       = ki;
    resp["kd"]       = kd;
    if (!ok) {
        resp["error"] = "set_gains_failed";
    }
    sendAck("CMD_DC_SET_VEL_GAINS", ok, resp);
}

// -----------------------------------------------------------------------------
// Telemetry
// -----------------------------------------------------------------------------
void CommandHandler::handleTelemSetInterval(JsonVariantConst payload) {
    uint32_t interval = payload["interval_ms"] | 0;

    DBG_PRINTF("[CMD] TELEM_SET_INTERVAL interval=%lu\n", (unsigned long)interval);
    telemetry_.setInterval(interval);

    JsonDocument resp;
    resp["interval_ms"] = interval;
    sendAck("CMD_TELEM_SET_INTERVAL", true, resp);
}

// -----------------------------------------------------------------------------
// Logging
// -----------------------------------------------------------------------------
void CommandHandler::handleSetLogLevel(JsonVariantConst payload) {
    const char* levelStr = payload["level"] | "info";

    DBG_PRINTF("[CMD] SET_LOG_LEVEL level=%s\n", levelStr);

    if (LoggingModule::instance()) {
        LoggingModule::instance()->setLogLevel(levelStr);
    }

    JsonDocument resp;
    resp["level"] = levelStr;
    sendAck("CMD_SET_LOG_LEVEL", true, resp);
}

// -----------------------------------------------------------------------------
// Loop Rates Commands
// -----------------------------------------------------------------------------
void CommandHandler::handleGetRates(JsonVariantConst) {
    LoopRates& r = getLoopRates();
    
    JsonDocument resp;
    resp["ctrl_hz"]    = r.ctrl_hz;
    resp["safety_hz"]  = r.safety_hz;
    resp["telem_hz"]   = r.telem_hz;
    resp["ctrl_ms"]    = r.ctrl_period_ms();
    resp["safety_ms"]  = r.safety_period_ms();
    resp["telem_ms"]   = r.telem_period_ms();
    sendAck("CMD_GET_RATES", true, resp);
}

void CommandHandler::handleCtrlSetRate(JsonVariantConst payload) {
    if (!requireIdle("CMD_CTRL_SET_RATE")) return;
    
    if (payload["hz"].isNull()) {
        sendError("CMD_CTRL_SET_RATE", "missing_hz");
        return;
    }
    
    uint16_t hz = static_cast<uint16_t>(payload["hz"].as<int>());
    if (hz == 0) {
        sendError("CMD_CTRL_SET_RATE", "invalid_hz");
        return;
    }
    
    bool inRange = clampHz(hz, LoopRates::CTRL_HZ_MIN, LoopRates::CTRL_HZ_MAX);
    getLoopRates().ctrl_hz = hz;
    
    JsonDocument resp;
    resp["applied_hz"] = hz;
    resp["in_range"]   = inRange;
    resp["ctrl_ms"]    = getLoopRates().ctrl_period_ms();
    sendAck("CMD_CTRL_SET_RATE", true, resp);
}

void CommandHandler::handleSafetySetRate(JsonVariantConst payload) {
    if (!requireIdle("CMD_SAFETY_SET_RATE")) return;
    
    if (payload["hz"].isNull()) {
        sendError("CMD_SAFETY_SET_RATE", "missing_hz");
        return;
    }
    
    uint16_t hz = static_cast<uint16_t>(payload["hz"].as<int>());
    if (hz == 0) {
        sendError("CMD_SAFETY_SET_RATE", "invalid_hz");
        return;
    }
    
    bool inRange = clampHz(hz, LoopRates::SAFETY_HZ_MIN, LoopRates::SAFETY_HZ_MAX);
    getLoopRates().safety_hz = hz;
    
    JsonDocument resp;
    resp["applied_hz"] = hz;
    resp["in_range"]   = inRange;
    resp["safety_ms"]  = getLoopRates().safety_period_ms();
    sendAck("CMD_SAFETY_SET_RATE", true, resp);
}

void CommandHandler::handleTelemSetRate(JsonVariantConst payload) {
    if (!requireIdle("CMD_TELEM_SET_RATE")) return;
    
    if (payload["hz"].isNull()) {
        sendError("CMD_TELEM_SET_RATE", "missing_hz");
        return;
    }
    
    uint16_t hz = static_cast<uint16_t>(payload["hz"].as<int>());
    if (hz == 0) {
        sendError("CMD_TELEM_SET_RATE", "invalid_hz");
        return;
    }
    
    bool inRange = clampHz(hz, LoopRates::TELEM_HZ_MIN, LoopRates::TELEM_HZ_MAX);
    getLoopRates().telem_hz = hz;
    
    JsonDocument resp;
    resp["applied_hz"] = hz;
    resp["in_range"]   = inRange;
    resp["telem_ms"]   = getLoopRates().telem_period_ms();
    sendAck("CMD_TELEM_SET_RATE", true, resp);
}

// -----------------------------------------------------------------------------
// Control Kernel - Signal Commands
// -----------------------------------------------------------------------------
void CommandHandler::handleCtrlSignalDefine(JsonVariantConst payload) {
    static constexpr const char* ACK = "CMD_CTRL_SIGNAL_DEFINE";
    Serial.println("[CMD] === CTRL_SIGNAL_DEFINE ===");
    Serial.printf("[CMD] controlModule_ = %p\n", (void*)controlModule_);
    Serial.printf("[CMD] Free heap: %u\n", ESP.getFreeHeap());

    DBG_PRINTLN("[CMD] CTRL_SIGNAL_DEFINE entry");

    if (!requireIdle(ACK)) return;

    if (!controlModule_) {
        DBG_PRINTLN("[CMD] CTRL_SIGNAL_DEFINE: no control module");
        sendError(ACK, "no_control_module");
        return;
    }

    // Validate required fields
    if (payload["id"].isNull() || payload["name"].isNull() || payload["kind"].isNull()) {
        DBG_PRINTLN("[CMD] CTRL_SIGNAL_DEFINE: missing fields");
        sendError(ACK, "missing_fields");
        return;
    }

    uint16_t id       = payload["id"].as<uint16_t>();
    const char* name  = payload["name"].as<const char*>();
    const char* kindS = payload["signal_kind"].as<const char*>();
    float initial     = payload["initial"] | 0.0f;

    DBG_PRINTF("[CMD] CTRL_SIGNAL_DEFINE: id=%u name=%s kind=%s initial=%.2f\n",
               id, name ? name : "null", kindS ? kindS : "null", initial);

    // Use helper from SignalBus.h
    SignalBus::Kind kind = signalKindFromString(kindS);
    

    DBG_PRINTLN("[CMD] CTRL_SIGNAL_DEFINE: calling signals().define()");
    bool ok = controlModule_->signals().define(id, name, kind, initial);
    DBG_PRINTF("[CMD] CTRL_SIGNAL_DEFINE: define() returned %d\n", (int)ok);

    JsonDocument resp;
    resp["id"]      = id;
    resp["name"]    = name ? name : "";
    resp["kind"]    = kindS ? kindS : "";
    resp["initial"] = initial;
    
    if (!ok) {
        resp["error"] = "define_failed";
    }
    sendAck(ACK, ok, resp);

    DBG_PRINTLN("[CMD] CTRL_SIGNAL_DEFINE: done");
}

void CommandHandler::handleCtrlSignalSet(JsonVariantConst payload) {
    static constexpr const char* ACK = "CMD_CTRL_SIGNAL_SET";

    if (!controlModule_) {
        sendError(ACK, "no_control_module");
        return;
    }
    
    uint16_t id = payload["id"] | 0;
    float value = payload["value"] | 0.0f;
    
    bool ok = controlModule_->signals().set(id, value, millis());
    
    JsonDocument resp;
    resp["id"] = id;
    resp["value"] = value;
    if (!ok) {
        resp["error"] = "signal_not_found";
    }
    sendAck(ACK, ok, resp);
}

void CommandHandler::handleCtrlSignalGet(JsonVariantConst payload) {
    static constexpr const char* ACK = "CMD_CTRL_SIGNAL_GET";

    if (!controlModule_) {
        sendError(ACK, "no_control_module");
        return;
    }
    
    uint16_t id = payload["id"] | 0;
    float value = 0.0f;
    bool ok = controlModule_->signals().get(id, value);
    
    JsonDocument resp;
    resp["id"] = id;
    resp["value"] = value;
    if (!ok) {
        resp["error"] = "signal_not_found";
    }
    sendAck(ACK, ok, resp);
}

void CommandHandler::handleCtrlSignalsList(JsonVariantConst) {
    static constexpr const char* ACK = "CMD_CTRL_SIGNALS_LIST";

    JsonDocument resp;
    
    if (!controlModule_) {
        resp["count"] = 0;
        resp["signals"].to<JsonArray>();
        sendAck(ACK, true, resp);
        return;
    }

    const auto& vec = controlModule_->signals().all();
    resp["count"] = static_cast<uint16_t>(vec.size());
    JsonArray arr = resp["signals"].to<JsonArray>();

    for (const auto& sdef : vec) {
        JsonObject s = arr.add<JsonObject>();
        s["id"]    = sdef.id;
        s["name"]  = sdef.name;
        s["kind"]  = signalKindToString(sdef.kind);
        s["value"] = sdef.value;
        s["ts_ms"] = sdef.ts_ms;
    }
    
    sendAck(ACK, true, resp);
}

/// -----------------------------------------------------------------------------
// Control Kernel - Slot Commands
// -----------------------------------------------------------------------------
void CommandHandler::handleCtrlSlotConfig(JsonVariantConst payload) {
    static constexpr const char* ACK = "CMD_CTRL_SLOT_CONFIG";

    if (!controlModule_) {
        sendError(ACK, "no_control_module");
        return;
    }
    
    SlotConfig cfg;
    cfg.slot = payload["slot"] | 0;
    cfg.rate_hz = payload["rate_hz"] | 100;
    cfg.require_armed = payload["require_armed"] | true;
    cfg.require_active = payload["require_active"] | true;
    
    const char* type = payload["controller_type"] | "PID";
    
    if (strcmp(type, "STATE_SPACE") == 0 || strcmp(type, "SS") == 0) {
        // State-space configuration
        cfg.ss_io.num_states = payload["num_states"] | 2;
        cfg.ss_io.num_inputs = payload["num_inputs"] | 1;
        
        // State signal IDs
        JsonArrayConst state_ids = payload["state_ids"].as<JsonArrayConst>();
        if (state_ids) {
            for (size_t i = 0; i < state_ids.size() && i < StateSpaceIO::MAX_STATES; i++) {
                cfg.ss_io.state_ids[i] = state_ids[i].as<uint16_t>();
            }
        }
        
        // Reference signal IDs
        JsonArrayConst ref_ids = payload["ref_ids"].as<JsonArrayConst>();
        if (ref_ids) {
            for (size_t i = 0; i < ref_ids.size() && i < StateSpaceIO::MAX_STATES; i++) {
                cfg.ss_io.ref_ids[i] = ref_ids[i].as<uint16_t>();
            }
        }
        
        // Output signal IDs
        JsonArrayConst out_ids = payload["output_ids"].as<JsonArrayConst>();
        if (out_ids) {
            for (size_t i = 0; i < out_ids.size() && i < StateSpaceIO::MAX_INPUTS; i++) {
                cfg.ss_io.output_ids[i] = out_ids[i].as<uint16_t>();
            }
        }
    } else {
        // PID configuration
        cfg.io.ref_id = payload["ref_id"] | 0;
        cfg.io.meas_id = payload["meas_id"] | 0;
        cfg.io.out_id = payload["out_id"] | 0;
    }
    
    bool ok = controlModule_->kernel().configureSlot(cfg, type);
    
    JsonDocument resp;
    resp["slot"] = cfg.slot;
    resp["controller_type"] = type;
    resp["rate_hz"] = cfg.rate_hz;
    if (!ok) {
        resp["error"] = "config_failed";
    }
    sendAck(ACK, ok, resp);
}

void CommandHandler::handleCtrlSlotEnable(JsonVariantConst payload) {
    static constexpr const char* ACK = "CMD_CTRL_SLOT_ENABLE";

    if (!controlModule_) {
        sendError(ACK, "no_control_module");
        return;
    }
    
    uint8_t slot = payload["slot"] | 0;
    bool enable = payload["enable"] | true;
    
    bool ok = controlModule_->kernel().enableSlot(slot, enable);
    
    JsonDocument resp;
    resp["slot"] = slot;
    resp["enable"] = enable;
    if (!ok) {
        resp["error"] = "enable_failed";
    }
    sendAck(ACK, ok, resp);
}

void CommandHandler::handleCtrlSlotReset(JsonVariantConst payload) {
    static constexpr const char* ACK = "CMD_CTRL_SLOT_RESET";

    if (!controlModule_) {
        sendError(ACK, "no_control_module");
        return;
    }
    
    uint8_t slot = payload["slot"] | 0;
    bool ok = controlModule_->kernel().resetSlot(slot);
    
    JsonDocument resp;
    resp["slot"] = slot;
    if (!ok) {
        resp["error"] = "reset_failed";
    }
    sendAck(ACK, ok, resp);
}

void CommandHandler::handleCtrlSlotSetParam(JsonVariantConst payload) {
    static constexpr const char* ACK = "CMD_CTRL_SLOT_SET_PARAM";

    if (!controlModule_) {
        sendError(ACK, "no_control_module");
        return;
    }

    uint8_t slot      = payload["slot"] | 0;
    const char* key   = payload["key"] | "";
    float value       = payload["value"] | 0.0f;

    if (!key || key[0] == '\0') {
        sendError(ACK, "missing_key");
        return;
    }

    bool ok = controlModule_->kernel().setParam(slot, key, value);

    JsonDocument resp;
    resp["slot"]  = slot;
    resp["key"]   = key;
    resp["value"] = value;
    if (!ok) {
        resp["error"] = "set_param_failed";
    }
    sendAck(ACK, ok, resp);
}

void CommandHandler::handleCtrlSlotSetParamArray(JsonVariantConst payload) {
    static constexpr const char* ACK = "CMD_CTRL_SLOT_SET_PARAM_ARRAY";

    if (!controlModule_) {
        sendError(ACK, "no_control_module");
        return;
    }
    
    uint8_t slot = payload["slot"] | 0;
    const char* key = payload["key"] | "";
    JsonArrayConst arr = payload["values"].as<JsonArrayConst>();
    
    if (!arr || arr.size() == 0) {
        JsonDocument resp;
        resp["slot"] = slot;
        resp["key"] = key;
        resp["error"] = "missing_values";
        sendAck(ACK, false, resp);
        return;
    }
    
    // Extract float array
    float values[12];  // Max size for K matrix (2x6)
    size_t len = std::min(arr.size(), sizeof(values)/sizeof(values[0]));
    for (size_t i = 0; i < len; i++) {
        values[i] = arr[i].as<float>();
    }
    
    bool ok = controlModule_->kernel().setParamArray(slot, key, values, len);
    
    JsonDocument resp;
    resp["slot"] = slot;
    resp["key"] = key;
    resp["count"] = (int)len;
    if (!ok) {
        resp["error"] = "set_param_array_failed";
    }
    sendAck(ACK, ok, resp);
}

void CommandHandler::handleCtrlSlotStatus(JsonVariantConst payload) {
    static constexpr const char* ACK = "CMD_CTRL_SLOT_STATUS";

    if (!controlModule_) {
        sendError(ACK, "no_control_module");
        return;
    }
    
    uint8_t slot = payload["slot"] | 0;
    
    auto cfg = controlModule_->kernel().getConfig(slot);
    auto st = controlModule_->kernel().getStatus(slot);
    
    JsonDocument resp;
    resp["slot"] = slot;
    resp["enabled"] = cfg.enabled;
    resp["rate_hz"] = cfg.rate_hz;
    resp["ok"] = st.ok;
    resp["run_count"] = st.run_count;
    resp["last_run_ms"] = st.last_run_ms;
    if (st.last_error) {
        resp["last_error"] = st.last_error;
    }
    sendAck(ACK, true, resp);
}