#include "core/CommandHandler.h"
#include "core/Messages.h"
#include "core/ModeManager.h"
#include "core/MotionController.h"
#include "core/SafetyManager.h"
#include "config/PinConfig.h"
#include "core/Event.h"
#include "modules/LoggingModule.h"   // <-- NEW
#include "managers/UltrasonicManager.h"
#include <Arduino.h>      // for strcmp, millis, etc.
#include "core/Debug.h"   // debug macros
#include "managers/UltrasonicManager.h"
#include "modules/TelemetryModule.h"
#include "managers/StepperManager.h"
#include "managers/EncoderManager.h"
#include "managers/DcMotorManager.h"


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
                               StepperManager&   stepper,
                               TelemetryModule&  telemetry,
                               UltrasonicManager& ultrasonic,
                               EncoderManager&   encoder,
                               DcMotorManager&   dc)
    : bus_(bus)
    , mode_(mode)
    , motion_(motion)
    , safety_(safety)
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
    DBG_PRINTLN("[CMD] CommandHandler::setup() subscribing (static)");
    // EventBus::Handler is now a plain function pointer: void (*)(const Event&)
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
    // Raw JSON from MessageRouter
    onJsonCommand(evt.payload.json);
}

void CommandHandler::onJsonCommand(const std::string& jsonStr) {

    // ----------------------------------------------------
    // Debug: show raw JSON received
    // ----------------------------------------------------
    DBG_PRINT("[CMD] raw JSON: ");
    DBG_PRINTLN(jsonStr.c_str());

    JsonMessage msg;
    if (!parseJsonToMessage(jsonStr, msg)) {
        DBG_PRINT("[CMD] Failed to parse JSON: ");
        DBG_PRINTLN(jsonStr.c_str());
        return;
    }

    // ----------------------------------------------------
    // Debug: show parsed message classification
    // ----------------------------------------------------
    DBG_PRINT("[CMD] kind=");
    DBG_PRINT((int)msg.kind);  // numeric enum value
    DBG_PRINT(" typeStr=");
    DBG_PRINT(msg.typeStr.c_str());
    DBG_PRINT(" cmdType=");
    DBG_PRINTLN((int)msg.cmdType);

    if (msg.kind != MsgKind::CMD) {
        DBG_PRINT("[CMD] Ignoring non-command JSON kind: ");
        DBG_PRINTLN(msg.typeStr.c_str());
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

        case CmdType::STEPPER_ENABLE:         handleStepperEnable(payload);      break;
        case CmdType::STEPPER_MOVE_REL:       handleStepperMoveRel(payload);     break;
        case CmdType::STEPPER_STOP:           handleStepperStop(payload);        break;

        // --- Ultrasonic ---
        case CmdType::ULTRASONIC_ATTACH:      handleUltrasonicAttach(payload);   break;
        case CmdType::ULTRASONIC_READ:        handleUltrasonicRead(payload);     break;

        // encoder 
        case CmdType::ENCODER_ATTACH:    handleEncoderAttach(payload);    break;
        case CmdType::ENCODER_READ:      handleEncoderRead(payload);      break;
        case CmdType::ENCODER_RESET:     handleEncoderReset(payload);     break;

        
        // --- DC motor control ---
        case CmdType::DC_SET_SPEED:           handleDcSetSpeed(payload);         break;
        case CmdType::DC_STOP:                handleDcStop(payload);             break;


        // Telemetry switch
        case CmdType::TELEM_SET_INTERVAL:     handleTelemSetInterval(payload);    break;

        // --- Logging control ---
        case CmdType::SET_LOG_LEVEL:          handleSetLogLevel(payload);        break;

        default:
            DBG_PRINT("[CMD] Unknown cmdType for typeStr=");
            DBG_PRINTLN(msg.typeStr.c_str());
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
        DBG_PRINT("[CMD] Unsupported mode string: ");
        DBG_PRINTLN(modeStr);
        return;
    }

    // If we're in ESTOP, only CLEAR_ESTOP is allowed to change state
    if (mode_.mode() == RobotMode::ESTOP) {
        DBG_PRINTLN("[CMD] Ignoring SET_MODE while in ESTOP");
        return;
    }

    mode_.setMode(newMode);
    // TODO: optionally publish a STATUS event with new mode
}

void CommandHandler::handleSetVel(JsonVariantConst payload) {
    // Basic safety gates
    if (!mode_.canMove() || safety_.isEstopActive()) {
        DBG_PRINTLN("[CMD] SET_VEL blocked by mode or ESTOP");
        return;
    }

    float vx    = payload["vx"]    | 0.0f;
    float omega = payload["omega"] | 0.0f;

    motion_.setVelocity(vx, omega);
}

// -----------------------------------------------------------------------------
// Logging control
// -----------------------------------------------------------------------------
void CommandHandler::handleSetLogLevel(JsonVariantConst payload) {
    const char* levelStr = payload["level"] | "info";

    if (LoggingModule::instance()) {
        LoggingModule::instance()->setLogLevel(levelStr);
    }

    JsonDocument resp;  // dont change these anymore this is the not depricated
    resp["src"]   = "mcu"; 
    resp["cmd"]   = "LOG_LEVEL_ACK";
    resp["level"] = levelStr;
    resp["ok"]    = true;

    std::string out;
    serializeJson(resp, out);

    Event evt;
    evt.type         = EventType::JSON_MESSAGE_TX;
    evt.payload.json = std::move(out);
    bus_.publish(evt);
}


// -----------------------------------------------------------------------------
// STOP / ESTOP / CLEAR_ESTOP / LED / etc.
// -----------------------------------------------------------------------------
void CommandHandler::handleStop() {
    DBG_PRINTLN("[CMD] STOP");
    motion_.stop();
}

void CommandHandler::handleEstop() {
    DBG_PRINTLN("[CMD] ESTOP");
    safety_.estop();
    motion_.stop();
    mode_.setMode(RobotMode::ESTOP);
}

void CommandHandler::handleClearEstop() {
    DBG_PRINTLN("[CMD] CLEAR_ESTOP");
    safety_.clearEstop();
    motion_.stop();
    mode_.setMode(RobotMode::IDLE);
}

void CommandHandler::handleLedOn() {
    DBG_PRINTLN("[CMD] LED ON");
    digitalWrite(Pins::LED_STATUS, HIGH);
}

void CommandHandler::handleLedOff() {
    DBG_PRINTLN("[CMD] LED OFF");
    digitalWrite(Pins::LED_STATUS, LOW);
}

// -----------------------------------------------------------------------------
// PWM
// -----------------------------------------------------------------------------
void CommandHandler::handlePwmSet(JsonVariantConst payload) {
    int   channel = payload["channel"] | 0;
    float duty    = payload["duty"]    | 0.0f;
    float freq    = payload["freq_hz"] | 0.0f;  // 0 = use default

    DBG_PRINTF("[CMD] PWM_SET ch=%d duty=%.3f freq=%.1f\n",
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

    uint8_t pin = 0;
    switch (servoId) {
        case 0:
            pin = Pins::SERVO1_SIG;
            break;
        default:
            DBG_PRINTF("[CMD] SERVO_ATTACH: unknown servoId=%d\n", servoId);
            return;
    }

    DBG_PRINTF("[CMD] SERVO_ATTACH id=%d pin=%d min=%dus max=%dus\n",
               servoId, pin, minUs, maxUs);

    servo_.attach(servoId, pin, minUs, maxUs);
}

void CommandHandler::handleServoDetach(JsonVariantConst payload) {
    int servoId = payload["servo_id"] | 0;

    DBG_PRINTF("[CMD] SERVO_DETACH id=%d\n", servoId);
    servo_.detach(servoId);
}

void CommandHandler::handleServoSetAngle(JsonVariantConst payload) {
    // Safety gate (good to keep this)
    if (!mode_.canMove() || safety_.isEstopActive()) {
        DBG_PRINTLN("[CMD] SERVO_SET_ANGLE blocked by mode or ESTOP");
        return;
    }

    int   servoId = payload["servo_id"]  | 0;
    float angle   = payload["angle_deg"] | 0.0f;
    int   durMs   = payload["duration_ms"] | 0;  // 0 = immediate

    DBG_PRINTF("[CMD] SERVO_SET_ANGLE id=%d angle=%.1f dur=%d ms\n",
               servoId, angle, durMs);

    if (durMs <= 0) {
        // Immediate move via ServoManager
        servo_.setAngle(servoId, angle);
    } else {
        // Smooth/interpolated motion via MotionController
        motion_.setServoTarget(servoId, angle, durMs);
    }
}

// -----------------------------------------------------------------------------
// Stepper
// -----------------------------------------------------------------------------
void CommandHandler::handleStepperMoveRel(JsonVariantConst payload) {
    // Safety gate
    if (!mode_.canMove() || safety_.isEstopActive()) {
        DBG_PRINTLN("[CMD] STEPPER_MOVE_REL blocked by mode or ESTOP");
        return;
    }

    int   motorId      = payload["motor_id"]      | 0;
    int   steps        = payload["steps"]         | 0;
    float speedSteps_s = payload["speed_steps_s"] | 1000.0f;

    DBG_PRINTF("[CMD] STEPPER_MOVE_REL motor=%d steps=%d speed=%.1f steps/s\n",
               motorId, steps, speedSteps_s);

    motion_.moveStepperRelative(motorId, steps, speedSteps_s);
}


void CommandHandler::handleStepperStop(JsonVariantConst payload) {
    int motorId = payload["motor_id"] | 0;

    DBG_PRINTF("[CMD] STEPPER_STOP motor=%d\n", motorId);

    // For now just call StepperManager directly; you could also add
    // a MotionController::stopStepper(...) wrapper if you want.
    stepper_.stop(motorId);
}

// Optional but very handy: enable/disable the stepper driver
void CommandHandler::handleStepperEnable(JsonVariantConst payload) {
    int  motorId = payload["motor_id"] | 0;
    bool enable  = payload["enable"]   | true;

    DBG_PRINTF("[CMD] STEPPER_ENABLE motor=%d enable=%d\n", motorId, (int)enable);

    // If you added MotionController::enableStepper(...):
    // motion_.enableStepper(motorId, enable);

    // Or call StepperManager directly:
    stepper_.setEnabled(motorId, enable);
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

    JsonDocument resp;
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

    DBG_PRINTF("[GPIO_WRITE] ch=%d val=%d ok=%d\n", ch, val, (int)ok);
}

void CommandHandler::handleGpioRead(JsonVariantConst payload) {
    int ch = payload["channel"] | -1;

    bool ok  = gpio_.hasChannel(ch);
    int  val = -1;
    if (ok) {
        val = gpio_.read(ch);
    }

    bool valOk = (val == 0 || val == 1);

    JsonDocument resp;
    resp["src"]     = "mcu";
    resp["cmd"]     = "GPIO_READ_ACK";
    resp["channel"] = ch;
    resp["value"]   = val;
    resp["ok"]      = ok && valOk;

    std::string out;
    serializeJson(resp, out);

    Event evt;
    evt.type         = EventType::JSON_MESSAGE_TX;
    evt.payload.json = out;
    bus_.publish(evt);

    DBG_PRINTF("[GPIO_READ] ch=%d val=%d ok=%d\n",
               ch, val, (int)(ok && valOk));
}

void CommandHandler::handleGpioToggle(JsonVariantConst payload) {
    int ch = payload["channel"] | -1;

    bool ok = gpio_.hasChannel(ch);
    if (ok) {
        gpio_.toggle(ch);
    }

    JsonDocument resp;
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

    DBG_PRINTF("[GPIO_TOGGLE] ch=%d ok=%d\n", ch, (int)ok);
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

    JsonDocument resp;
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

    DBG_PRINTF("[GPIO_REGISTER_CHANNEL] ch=%d pin=%d mode=%s ok=%d\n",
               ch, pin, modeStr, (int)ok);
}
// -----------------------------------------------------------------------------
// Sensor groups
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// ultrasonic sensor – write / read / toggle / register-channel
// -----------------------------------------------------------------------------

void CommandHandler::handleUltrasonicAttach(JsonVariantConst payload) {
    int sensorId = payload["sensor_id"] | 0;

    uint8_t trigPin = 0;
    uint8_t echoPin = 0;

    switch (sensorId) {
        case 0:
            trigPin = Pins::ULTRA0_TRIG;  // = 5 from your JSON
            echoPin = Pins::ULTRA0_ECHO;  // = 4
            break;

        default:
            DBG_PRINTF("[CMD] ULTRASONIC_ATTACH: unknown sensorId=%d\n", sensorId);
            return;
    }

    DBG_PRINTF("[CMD] ULTRASONIC_ATTACH id=%d trigPin=%d echoPin=%d\n",
               sensorId, trigPin, echoPin);

    bool ok = ultrasonic_.attach(sensorId, trigPin, echoPin);

    JsonDocument resp;
    resp["src"]        = "mcu";
    resp["cmd"]        = "ULTRASONIC_ATTACH_ACK";
    resp["sensor_id"]  = sensorId;
    resp["trig_pin"]   = trigPin;
    resp["echo_pin"]   = echoPin;
    resp["ok"]         = ok;

    std::string out;
    serializeJson(resp, out);

    Event evt;
    evt.type         = EventType::JSON_MESSAGE_TX;
    evt.payload.json = std::move(out);
    bus_.publish(evt);
}

void CommandHandler::handleUltrasonicRead(JsonVariantConst payload) {
    int sensorId = payload["sensor_id"] | 0;

    DBG_PRINTF("[CMD] ULTRASONIC_READ id=%d\n", sensorId);

    float distCm = ultrasonic_.readDistanceCm(sensorId);
    bool ok = distCm >= 0.0f;

    JsonDocument resp;
    resp["src"]        = "mcu";
    resp["cmd"]        = "ULTRASONIC_READ_ACK";
    resp["sensor_id"]  = sensorId;
    resp["distance_cm"] = ok ? distCm : -1.0f;
    resp["ok"]         = ok;

    std::string out;
    serializeJson(resp, out);

    Event evt;
    evt.type         = EventType::JSON_MESSAGE_TX;
    evt.payload.json = std::move(out);
    bus_.publish(evt);

    DBG_PRINTF("[ULTRA_READ] id=%d dist=%.2f cm ok=%d\n",
               sensorId, distCm, (int)ok);
}


// -----------------------------------------------------------------------------
// ultrasonic sensor – write / read / toggle / register-channel
// -----------------------------------------------------------------------------
void CommandHandler::handleTelemSetInterval(JsonVariantConst payload) {
    uint32_t interval = payload["interval_ms"] | 0;
    telemetry_.setInterval(interval);
    // (optional) send TELEM_SET_INTERVAL_ACK back
}

// -----------------------------------------------------------------------------
// Encoder – attach/read via runtime pins from host
// -----------------------------------------------------------------------------
void CommandHandler::handleEncoderAttach(JsonVariantConst payload) {
    int encoderId = payload["encoder_id"] | 0;
    int pinA      = payload["pin_a"]      | Pins::ENC0_A;
    int pinB      = payload["pin_b"]      | Pins::ENC0_B;

    // Call the manager. If attach() is void, just call it.
    encoder_.attach(
        static_cast<uint8_t>(encoderId),
        static_cast<gpio_num_t>(pinA),
        static_cast<gpio_num_t>(pinB)
    );

    bool ok = true;  // assume success; change if you later make attach() return bool

    JsonDocument resp;
    resp["src"]        = "mcu";
    resp["cmd"]        = "ENCODER_ATTACH_ACK";
    resp["encoder_id"] = encoderId;
    resp["pin_a"]      = pinA;
    resp["pin_b"]      = pinB;
    resp["ok"]         = ok;

    std::string out;
    serializeJson(resp, out);

    Event evt;
    evt.type         = EventType::JSON_MESSAGE_TX;
    evt.payload.json = std::move(out);
    bus_.publish(evt);
}

void CommandHandler::handleEncoderRead(JsonVariantConst payload) {
    int encoderId = payload["encoder_id"] | 0;

    int32_t ticks = encoder_.getCount(static_cast<uint8_t>(encoderId));

    JsonDocument resp;
    resp["src"]        = "mcu";
    resp["cmd"]        = "ENCODER_READ_ACK";
    resp["encoder_id"] = encoderId;
    resp["ticks"]      = ticks;

    std::string out;
    serializeJson(resp, out);

    Event evt;
    evt.type         = EventType::JSON_MESSAGE_TX;
    evt.payload.json = std::move(out);
    bus_.publish(evt);
}

void CommandHandler::handleEncoderReset(JsonVariantConst payload) {
    int encoderId = payload["encoder_id"] | 0;

    encoder_.reset(static_cast<uint8_t>(encoderId));

    JsonDocument resp;
    resp["src"]        = "mcu";
    resp["cmd"]        = "ENCODER_RESET_ACK";
    resp["encoder_id"] = encoderId;
    resp["ok"]         = true;

    std::string out;
    serializeJson(resp, out);

    Event evt;
    evt.type         = EventType::JSON_MESSAGE_TX;
    evt.payload.json = std::move(out);
    bus_.publish(evt);
}

// DC motor stuff
void CommandHandler::handleDcSetSpeed(JsonVariantConst payload) {
    int   motorId = payload["motor_id"] | 0;
    float speed   = payload["speed"]    | 0.0f;  // -1.0 .. +1.0

    DBG_PRINTF("[CMD] DC_SET_SPEED motor=%d speed=%.3f\n", motorId, speed);

    bool ok = dc_.setSpeed(
        static_cast<uint8_t>(motorId),
        speed
    );

    using namespace ArduinoJson;
    JsonDocument resp;
    resp["src"]      = "mcu";
    resp["cmd"]      = "DC_SET_SPEED_ACK";
    resp["motor_id"] = motorId;
    resp["speed"]    = speed;
    resp["ok"]       = ok;

    std::string out;
    serializeJson(resp, out);

    Event evt;
    evt.type         = EventType::JSON_MESSAGE_TX;
    evt.payload.json = std::move(out);
    bus_.publish(evt);
}

void CommandHandler::handleDcStop(JsonVariantConst payload) {
    int motorId = payload["motor_id"] | 0;

    DBG_PRINTF("[CMD] DC_STOP motor=%d\n", motorId);

    dc_.stop(static_cast<uint8_t>(motorId));

    using namespace ArduinoJson;
    JsonDocument resp;
    resp["src"]      = "mcu";
    resp["cmd"]      = "DC_STOP_ACK";
    resp["motor_id"] = motorId;
    resp["ok"]       = true;

    std::string out;
    serializeJson(resp, out);

    Event evt;
    evt.type         = EventType::JSON_MESSAGE_TX;
    evt.payload.json = std::move(out);
    bus_.publish(evt);
}
