#include "core/CommandHandler.h"
#include "core/Messages.h"
#include "core/ModeManager.h"
#include "core/MotionController.h"
#include "core/SafetyManager.h"
#include "config/PinConfig.h"
#include "core/Event.h"

#include <Arduino.h>  // for strcmp, millis, Serial, etc.

using namespace ArduinoJson;

CommandHandler* CommandHandler::s_instance = nullptr;

// -----------------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------------
CommandHandler::CommandHandler(EventBus&         bus,
                               ModeManager&      mode,
                               MotionController& motion,
                               SafetyManager&    safety,
                               GpioManager&      gpio,
                               PwmManager&       pwm,
                               ServoManager&     servo,
                               StepperManager&   stepper)
    : bus_(bus)
    , mode_(mode)
    , motion_(motion)
    , safety_(safety)
    , gpio_(gpio)
    , pwm_(pwm)
    , servo_(servo)
    , stepper_(stepper)
{
    s_instance = this;
}

// -----------------------------------------------------------------------------
// Setup / Event subscription
// -----------------------------------------------------------------------------
void CommandHandler::setup() {
    Serial.println("[CMD] CommandHandler::setup() subscribing (static)");
    // EventBus::Handler is now a plain function pointer: void (*)(const Event&)
    bus_.subscribe(&CommandHandler::handleEventStatic);
    Serial.println("[CMD] CommandHandler::setup() done");
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
    // Raw JSON from MessageRouter
    onJsonCommand(evt.payload.json);
}

void CommandHandler::onJsonCommand(const std::string& jsonStr) {

    // ----------------------------------------------------
    // Debug: show raw JSON received
    // ----------------------------------------------------
    Serial.print("[CMD] raw JSON: ");
    Serial.println(jsonStr.c_str());

    JsonMessage msg;
    if (!parseJsonToMessage(jsonStr, msg)) {
        Serial.print("[CMD] Failed to parse JSON: ");
        Serial.println(jsonStr.c_str());
        return;
    }

    // ----------------------------------------------------
    // Debug: show parsed message classification
    // ----------------------------------------------------
    Serial.print("[CMD] kind=");    
    Serial.print((int)msg.kind);  // numeric enum value
    Serial.print(" typeStr=");    
    Serial.print(msg.typeStr.c_str());
    Serial.print(" cmdType=");    
    Serial.println((int)msg.cmdType);

    if (msg.kind != MsgKind::CMD) {
        Serial.print("[CMD] Ignoring non-command JSON kind: ");
        Serial.println(msg.typeStr.c_str());
        return;
    }

    // Payload as read-only view
    JsonVariantConst payload = msg.payload.as<JsonVariantConst>();

    switch (msg.cmdType) {
        // --- Core robot behavior ---
        case CmdType::SET_MODE:               handleSetMode(payload);            break;
        case CmdType::SET_VEL:                handleSetVel(payload);             break;
        case CmdType::STOP:                   handleStop();                      break;
        case CmdType::ESTOP:                  handleEstop();                     break;
        case CmdType::CLEAR_ESTOP:            handleClearEstop();                break;

        // --- Simple LED control ---
        case CmdType::LED_ON:                 handleLedOn();                     break;
        case CmdType::LED_OFF:                handleLedOff();                    break;

        // --- GPIO / PWM / Servo / Stepper ---
        case CmdType::GPIO_WRITE:             handleGpioWrite(payload);          break;
        case CmdType::GPIO_READ:              handleGpioRead(payload);           break;
        case CmdType::GPIO_TOGGLE:            handleGpioToggle(payload);         break;
        case CmdType::GPIO_REGISTER_CHANNEL:  handleGpioRegisterChannel(payload);break;

        case CmdType::PWM_SET:                handlePwmSet(payload);             break;

        case CmdType::SERVO_ATTACH:           handleServoAttach(payload);        break;
        case CmdType::SERVO_DETACH:           handleServoDetach(payload);        break;
        case CmdType::SERVO_SET_ANGLE:        handleServoSetAngle(payload);      break;

        case CmdType::STEPPER_MOVE_REL:       handleStepperMoveRel(payload);     break;
        case CmdType::STEPPER_STOP:           handleStepperStop(payload);        break;

        default:
            Serial.print("[CMD] Unknown cmdType for typeStr=");
            Serial.println(msg.typeStr.c_str());
            break;
    }
}

// -----------------------------------------------------------------------------
// Core robot behaviors
// -----------------------------------------------------------------------------
void CommandHandler::handleSetMode(JsonVariantConst payload) {
    const char* modeStr = payload["mode"] | "IDLE";

    RobotMode newMode = mode_.mode();

    if (strcmp(modeStr, "IDLE") == 0) {
        newMode = RobotMode::IDLE;
    } else if (strcmp(modeStr, "ARMED") == 0) {
        newMode = RobotMode::ARMED;
    } else if (strcmp(modeStr, "ACTIVE") == 0) {
        newMode = RobotMode::ACTIVE;
    } else {
        Serial.print("[CMD] Unsupported mode string: ");
        Serial.println(modeStr);
        return;
    }

    // If we're in ESTOP, only CLEAR_ESTOP is allowed to change state
    if (mode_.mode() == RobotMode::ESTOP) {
        Serial.println("[CMD] Ignoring SET_MODE while in ESTOP");
        return;
    }

    mode_.setMode(newMode);
    // TODO: optionally publish a STATUS event with new mode
}

void CommandHandler::handleSetVel(JsonVariantConst payload) {
    // Basic safety gates
    if (!mode_.canMove() || safety_.isEstopActive()) {
        Serial.println("[CMD] SET_VEL blocked by mode or ESTOP");
        return;
    }

    float vx    = payload["vx"]    | 0.0f;
    float omega = payload["omega"] | 0.0f;

    // Optional clamping
    // vx    = constrain(vx, -0.5f, 0.5f);
    // omega = constrain(omega, -1.0f, 1.0f);

    motion_.setVelocity(vx, omega);
    // TODO: later add velocity command timeout safety
}

void CommandHandler::handleStop() {
    Serial.println("[CMD] STOP");
    motion_.stop();
}

void CommandHandler::handleEstop() {
    Serial.println("[CMD] ESTOP");
    safety_.estop();
    motion_.stop();
    mode_.setMode(RobotMode::ESTOP);
    // TODO: publish STATUS_ESTOP if desired
}

void CommandHandler::handleClearEstop() {
    Serial.println("[CMD] CLEAR_ESTOP");
    safety_.clearEstop();
    motion_.stop();
    mode_.setMode(RobotMode::IDLE);
}

void CommandHandler::handleLedOn() {
    Serial.println("[CMD] LED ON");
    digitalWrite(Pins::LED_STATUS, HIGH);
}

void CommandHandler::handleLedOff() {
    Serial.println("[CMD] LED OFF");
    digitalWrite(Pins::LED_STATUS, LOW);
}

// -----------------------------------------------------------------------------
// PWM
// -----------------------------------------------------------------------------
void CommandHandler::handlePwmSet(JsonVariantConst payload) {
    int   channel = payload["channel"] | 0;
    float duty    = payload["duty"]    | 0.0f;
    float freq    = payload["freq_hz"] | 0.0f;  // 0 = use default

    Serial.printf("[CMD] PWM_SET ch=%d duty=%.3f freq=%.1f\n",
                  channel, duty, freq);

    pwm_.set(channel, duty, freq);
}

// -----------------------------------------------------------------------------
// Servo
// -----------------------------------------------------------------------------
void CommandHandler::handleServoAttach(JsonVariantConst payload) {
    int servoId  = payload["servo_id"] | 0;
    int minUs    = payload["min_us"]   | 1000;
    int maxUs    = payload["max_us"]   | 2000;

    // Map servoId -> actual GPIO pin from Pins::
    uint8_t pin = 0;
    switch (servoId) {
        case 0:
            pin = Pins::SERVO1_SIG;   // <-- 18 from PinConfig.h
            break;

        // later you can add:
        // case 1: pin = Pins::SERVO2_SIG; break;
        // case 2: pin = Pins::SERVO3_SIG; break;
        default:
            Serial.printf("[CMD] SERVO_ATTACH: unknown servoId=%d\n", servoId);
            return;
    }

    Serial.printf("[CMD] SERVO_ATTACH id=%d pin=%d min=%dus max=%dus\n",
                  servoId, pin, minUs, maxUs);

    servo_.attach(servoId, pin, minUs, maxUs);
}

void CommandHandler::handleServoDetach(JsonVariantConst payload) {
    int servoId = payload["servo_id"] | 0;

    Serial.printf("[CMD] SERVO_DETACH id=%d\n", servoId);
    servo_.detach(servoId);
}

void CommandHandler::handleServoSetAngle(JsonVariantConst payload) {
    int   servoId = payload["servo_id"]  | 0;
    float angle   = payload["angle_deg"] | 0.0f;

    Serial.printf("[CMD] SERVO_SET_ANGLE id=%d angle=%.1f\n",
                  servoId, angle);

    servo_.setAngle(servoId, angle);
}

// -----------------------------------------------------------------------------
// Stepper
// -----------------------------------------------------------------------------
void CommandHandler::handleStepperMoveRel(JsonVariantConst payload) {
    int   motorId      = payload["motor_id"]      | 0;
    int   steps        = payload["steps"]         | 0;
    float speedSteps_s = payload["speed_steps_s"] | 1000.0f;

    Serial.printf("[CMD] STEPPER_MOVE_REL motor=%d steps=%d speed=%.1f steps/s\n",
                  motorId, steps, speedSteps_s);

    stepper_.moveRelative(motorId, steps, speedSteps_s);
}

void CommandHandler::handleStepperStop(JsonVariantConst payload) {
    int motorId = payload["motor_id"] | 0;

    Serial.printf("[CMD] STEPPER_STOP motor=%d\n", motorId);
    stepper_.stop(motorId);
}

// -----------------------------------------------------------------------------
// GPIO – write / read / toggle / register-channel
// -----------------------------------------------------------------------------
void CommandHandler::handleGpioWrite(JsonVariantConst payload) {
    int ch  = payload["channel"] | -1;
    int val = payload["value"]   | 0;

    bool ok = gpio_.hasChannel(ch);
    if (ok) {
        gpio_.write(ch, val);
    }

    DynamicJsonDocument resp(256);
    resp["src"]     = "mcu";
    resp["cmd"]     = "GPIO_WRITE_ACK";
    resp["channel"] = ch;
    resp["value"]   = val;
    resp["ok"]      = ok;

    std::string out;
    serializeJson(resp, out);

    Event evt;
    evt.type         = EventType::JSON_MESSAGE_TX;
    evt.payload.json = out;
    bus_.publish(evt);

    Serial.printf("[GPIO_WRITE] ch=%d val=%d ok=%d\n", ch, val, (int)ok);
}

void CommandHandler::handleGpioRead(JsonVariantConst payload) {
    int ch = payload["channel"] | -1;

    bool ok  = gpio_.hasChannel(ch);
    int  val = -1;
    if (ok) {
        val = gpio_.read(ch);
    }

    bool valOk = (val == 0 || val == 1);

    DynamicJsonDocument resp(256);
    resp["src"]     = "mcu";
    resp["cmd"]     = "GPIO_READ_ACK";
    resp["channel"] = ch;
    resp["value"]   = val;   // -1 means unknown/error
    resp["ok"]      = ok && valOk;

    std::string out;
    serializeJson(resp, out);

    Event evt;
    evt.type         = EventType::JSON_MESSAGE_TX;
    evt.payload.json = out;
    bus_.publish(evt);

    Serial.printf("[GPIO_READ] ch=%d val=%d ok=%d\n", ch, val, (int)(ok && valOk));
}

void CommandHandler::handleGpioToggle(JsonVariantConst payload) {
    int ch = payload["channel"] | -1;

    bool ok = gpio_.hasChannel(ch);
    if (ok) {
        gpio_.toggle(ch);
    }

    DynamicJsonDocument resp(256);
    resp["src"]     = "mcu";
    resp["cmd"]     = "GPIO_TOGGLE_ACK";
    resp["channel"] = ch;
    resp["ok"]      = ok;

    std::string out;
    serializeJson(resp, out);

    Event evt;
    evt.type         = EventType::JSON_MESSAGE_TX;
    evt.payload.json = out;
    bus_.publish(evt);

    Serial.printf("[GPIO_TOGGLE] ch=%d ok=%d\n", ch, (int)ok);
}

void CommandHandler::handleGpioRegisterChannel(JsonVariantConst payload) {
    // {
    //   "cmd": "CMD_GPIO_REGISTER_CHANNEL",
    //   "channel": 0,
    //   "pin": 2,
    //   "mode": "output" | "input" | "input_pullup"
    // }

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

    DynamicJsonDocument resp(256);
    resp["src"]     = "mcu";
    resp["cmd"]     = "GPIO_REGISTER_CHANNEL_ACK";
    resp["channel"] = ch;
    resp["pin"]     = pin;
    resp["mode"]    = modeStr;
    resp["ok"]      = ok;

    std::string out;
    serializeJson(resp, out);

    Event evt;
    evt.type         = EventType::JSON_MESSAGE_TX;
    evt.payload.json = out;
    bus_.publish(evt);

    Serial.printf("[GPIO_REGISTER_CHANNEL] ch=%d pin=%d mode=%s ok=%d\n",
                  ch, pin, modeStr, (int)ok);
}
