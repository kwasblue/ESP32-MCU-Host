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
    
    if (msg.kind != MsgKind::CMD) {
        DBG_PRINT("[CMD] Ignoring non-command: ");
        DBG_PRINTLN(msg.typeStr.c_str());
        return;
    }

    // Set current context for ACK/Error helpers
    currentSeq_ = msg.seq;
    currentCmdType_ = msg.cmdType;

    // If this exact command was already processed (retry), replay previous ACK and exit.
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
    // If seq is missing/zero, skip replay (optional policy)
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
    // Optional policy: don't cache seq==0 commands
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

    // NEW: seq echo so host can match this ACK to the request
    resp["seq"] = currentSeq_;

    std::string out;
    serializeJson(resp, out);

    // NEW: cache for retry replay (prevents double execution)
    storeAck(currentCmdType_, currentSeq_, out);

    publishJson(std::move(out));
}

void CommandHandler::sendError(const char* cmd, const char* error) {
    JsonDocument resp;
    resp["src"]   = "mcu";
    resp["cmd"]   = cmd;
    resp["ok"]    = false;
    resp["error"] = error;

    // NEW: seq echo for errors too
    resp["seq"] = currentSeq_;

    std::string out;
    serializeJson(resp, out);

    // NEW: cache error too (retry should return same error)
    storeAck(currentCmdType_, currentSeq_, out);

    publishJson(std::move(out));
}


// -----------------------------------------------------------------------------
// Safety / State Machine Commands
// -----------------------------------------------------------------------------
void CommandHandler::handleHeartbeat() {
    DBG_PRINTLN("[CMD] HEARTBEAT");
    mode_.onHostHeartbeat(millis());

    JsonDocument resp;
    resp["state"] = robotModeToString(mode_.mode());
    sendAck("HEARTBEAT_ACK", true, resp);
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
    sendAck("ARM_ACK", ok, resp);
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
    sendAck("DISARM_ACK", ok, resp);
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
    sendAck("ACTIVATE_ACK", ok, resp);
}

void CommandHandler::handleDeactivate() {
    DBG_PRINTLN("[CMD] DEACTIVATE");
    mode_.deactivate();

    JsonDocument resp;
    resp["state"] = robotModeToString(mode_.mode());
    sendAck("DEACTIVATE_ACK", true, resp);
}

void CommandHandler::handleEstop() {
    DBG_PRINTLN("[CMD] ESTOP");
    mode_.estop();

    JsonDocument resp;
    resp["state"] = robotModeToString(mode_.mode());
    sendAck("ESTOP_ACK", true, resp);
}

void CommandHandler::handleClearEstop() {
    DBG_PRINTLN("[CMD] CLEAR_ESTOP");
    bool cleared = mode_.clearEstop();

    JsonDocument resp;
    resp["state"] = robotModeToString(mode_.mode());
    if (!cleared) {
        resp["error"] = "cannot_clear";
    }
    sendAck("CLEAR_ESTOP_ACK", cleared, resp);
}

void CommandHandler::handleStop() {
    DBG_PRINTLN("[CMD] STOP");
    motion_.stop();

    JsonDocument resp;
    sendAck("STOP_ACK", true, resp);
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
    sendAck("SET_MODE_ACK", ok, resp);
}

// -----------------------------------------------------------------------------
// Motion
// -----------------------------------------------------------------------------
void CommandHandler::handleSetVel(JsonVariantConst payload) {
    if (!mode_.canMove()) {
        sendError("SET_VEL_ACK", "not_armed");
        return;
    }

    float vx    = payload["vx"]    | 0.0f;
    float omega = payload["omega"] | 0.0f;
    float safe_vx, safe_omega;

    if (!mode_.validateVelocity(vx, omega, safe_vx, safe_omega)) {
        sendError("SET_VEL_ACK", "invalid_velocity");
        return;
    }

    DBG_PRINTF("[CMD] SET_VEL vx=%.3f omega=%.3f\n", safe_vx, safe_omega);

    mode_.onMotionCommand(millis());
    motion_.setVelocity(safe_vx, safe_omega);

    JsonDocument resp;
    resp["vx"]    = safe_vx;
    resp["omega"] = safe_omega;
    sendAck("SET_VEL_ACK", true, resp);
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
    sendAck("LED_ON_ACK", true, resp);
}

void CommandHandler::handleLedOff() {
    DBG_PRINTLN("[CMD] LED_OFF");
    digitalWrite(Pins::LED_STATUS, LOW);

    JsonDocument resp;
    resp["pin"] = Pins::LED_STATUS;
    resp["on"]  = false;
    sendAck("LED_OFF_ACK", true, resp);
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
    sendAck("GPIO_WRITE_ACK", ok, resp);
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
    sendAck("GPIO_READ_ACK", ok, resp);
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
    sendAck("GPIO_TOGGLE_ACK", ok, resp);
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
    sendAck("GPIO_REGISTER_CHANNEL_ACK", ok, resp);
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
    sendAck("PWM_SET_ACK", true, resp);
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
    sendAck("SERVO_ATTACH_ACK", ok, resp);
}

void CommandHandler::handleServoDetach(JsonVariantConst payload) {
    int servoId = payload["servo_id"] | 0;

    DBG_PRINTF("[CMD] SERVO_DETACH id=%d\n", servoId);
    servo_.detach(servoId);

    JsonDocument resp;
    resp["servo_id"] = servoId;
    sendAck("SERVO_DETACH_ACK", true, resp);
}

void CommandHandler::handleServoSetAngle(JsonVariantConst payload) {
    if (!mode_.canMove()) {
        sendError("SERVO_SET_ANGLE_ACK", "not_armed");
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
    sendAck("SERVO_SET_ANGLE_ACK", true, resp);
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
    sendAck("STEPPER_ENABLE_ACK", true, resp);
}

void CommandHandler::handleStepperMoveRel(JsonVariantConst payload) {
    if (!mode_.canMove()) {
        sendError("STEPPER_MOVE_REL_ACK", "not_armed");
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
    sendAck("STEPPER_MOVE_REL_ACK", true, resp);
}

void CommandHandler::handleStepperStop(JsonVariantConst payload) {
    int motorId = payload["motor_id"] | 0;

    DBG_PRINTF("[CMD] STEPPER_STOP motor=%d\n", motorId);
    stepper_.stop(motorId);

    JsonDocument resp;
    resp["motor_id"] = motorId;
    sendAck("STEPPER_STOP_ACK", true, resp);
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
    sendAck("ULTRASONIC_ATTACH_ACK", ok, resp);
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
    sendAck("ULTRASONIC_READ_ACK", ok, resp);
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
    sendAck("ENCODER_ATTACH_ACK", true, resp);
}

void CommandHandler::handleEncoderRead(JsonVariantConst payload) {
    int encoderId = payload["encoder_id"] | 0;

    int32_t ticks = encoder_.getCount(static_cast<uint8_t>(encoderId));

    DBG_PRINTF("[CMD] ENCODER_READ id=%d ticks=%ld\n", encoderId, (long)ticks);

    JsonDocument resp;
    resp["encoder_id"] = encoderId;
    resp["ticks"]      = ticks;
    sendAck("ENCODER_READ_ACK", true, resp);
}

void CommandHandler::handleEncoderReset(JsonVariantConst payload) {
    int encoderId = payload["encoder_id"] | 0;

    DBG_PRINTF("[CMD] ENCODER_RESET id=%d\n", encoderId);
    encoder_.reset(static_cast<uint8_t>(encoderId));

    JsonDocument resp;
    resp["encoder_id"] = encoderId;
    sendAck("ENCODER_RESET_ACK", true, resp);
}

// -----------------------------------------------------------------------------
// DC Motor
// -----------------------------------------------------------------------------
void CommandHandler::handleDcSetSpeed(JsonVariantConst payload) {
    if (!mode_.canMove()) {
        sendError("DC_SET_SPEED_ACK", "not_armed");
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
    sendAck("DC_SET_SPEED_ACK", ok, resp);
}

void CommandHandler::handleDcStop(JsonVariantConst payload) {
    int motorId = payload["motor_id"] | 0;

    DBG_PRINTF("[CMD] DC_STOP motor=%d\n", motorId);
    dc_.stop(static_cast<uint8_t>(motorId));

    JsonDocument resp;
    resp["motor_id"] = motorId;
    sendAck("DC_STOP_ACK", true, resp);
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
    sendAck("DC_VEL_PID_ENABLE_ACK", ok, resp);
}

void CommandHandler::handleDcSetVelTarget(JsonVariantConst payload) {
    if (!mode_.canMove()) {
        sendError("DC_SET_VEL_TARGET_ACK", "not_armed");
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
    sendAck("DC_SET_VEL_TARGET_ACK", ok, resp);
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
    sendAck("DC_SET_VEL_GAINS_ACK", ok, resp);
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
    sendAck("TELEM_SET_INTERVAL_ACK", true, resp);
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
    sendAck("SET_LOG_LEVEL_ACK", true, resp);
}
