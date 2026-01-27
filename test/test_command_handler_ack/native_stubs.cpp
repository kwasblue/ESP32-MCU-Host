// test/test_command_handler_ack/native_stubs.cpp
//
// Native-only stubs to satisfy the linker when we include CommandHandler.cpp
// directly (and do NOT compile the full embedded src tree).

#include <cstdint>

#include "core/ModeManager.h"
#include "modules/LoggingModule.h"
#include "modules/TelemetryModule.h"
#include "managers/EncoderManager.h"
#include "core/MotionController.h"

// -----------------------------------------------------------------------------
// robotModeToString()
// -----------------------------------------------------------------------------
const char* robotModeToString(RobotMode) {
    return "UNKNOWN";
}

// -----------------------------------------------------------------------------
// ModeManager stubs (match ModeManager.h exactly)
// -----------------------------------------------------------------------------
void ModeManager::onHostHeartbeat(unsigned int) {}
void ModeManager::onMotionCommand(unsigned int) {}

bool ModeManager::validateVelocity(float vx, float omega, float& outVx, float& outOmega) {
    outVx = vx;
    outOmega = omega;
    return true;
}

void ModeManager::arm() {}
void ModeManager::activate() {}
void ModeManager::deactivate() {}
void ModeManager::disarm() {}
void ModeManager::estop() {}

// IMPORTANT: your header says bool clearEstop()
bool ModeManager::clearEstop() { return true; }

// -----------------------------------------------------------------------------
// LoggingModule stubs
// -----------------------------------------------------------------------------
LoggingModule* LoggingModule::s_instance = nullptr;

void LoggingModule::setLogLevel(const char* level) {
    (void)level;
}

// If you get an undefined symbol for LoggingModule::instance(), uncomment:
//
// LoggingModule& LoggingModule::instance() {
//     static LoggingModule inst;
//     s_instance = &inst;
//     return inst;
// }

// -----------------------------------------------------------------------------
// EncoderManager stubs (match EncoderManager.h exactly)
// -----------------------------------------------------------------------------
EncoderManager::EncoderManager() {}

void EncoderManager::attach(uint8_t, uint8_t, uint8_t) {}

void EncoderManager::reset(uint8_t) {}

int32_t EncoderManager::getCount(uint8_t) const {
    return 0;
}

// -----------------------------------------------------------------------------
// TelemetryModule stubs
//
// DO NOT define ~TelemetryModule() unless TelemetryModule.h DECLARES it.
// Your compiler error shows it is *implicitly declared*, so you must NOT
// define it here.
//
// Fixing the vtable issue:
// - TelemetryModule must have at least one non-inline virtual function declared
//   in TelemetryModule.h (the "key function") whose definition normally lives
//   in TelemetryModule.cpp.
// - You MUST stub that exact function signature here.
//
// Steps:
// 1) Open include/modules/TelemetryModule.h
// 2) Find TelemetryModule's virtual methods declared with ';' (not inline bodies)
// 3) Copy/paste those declarations and implement empty bodies here.
//
// Common examples are setup(), loop(), update(), handleEvent(), etc — but you
// MUST match your header EXACTLY.
// -----------------------------------------------------------------------------
TelemetryModule::TelemetryModule(EventBus& bus)
    : bus_(bus) {}

void TelemetryModule::setup() {
    // no-op for native tests
}

void TelemetryModule::loop(uint32_t now_ms) {
    (void)now_ms;
    // no-op for native tests
}

void TelemetryModule::registerProvider(const char* name, JsonProviderFn fn) {
    (void)name;
    (void)fn;
    // no-op for native tests
}

void TelemetryModule::registerBinProvider(uint8_t section_id, BinProviderFn fn) {
    (void)section_id;
    (void)fn;
    // no-op for native tests
}

void TelemetryModule::setInterval(uint32_t intervalMs) {
    (void)intervalMs;
    // no-op for native tests
}

void TelemetryModule::setBinaryEnabled(bool en) { (void)en; }
void TelemetryModule::setJsonEnabled(bool en)   { (void)en; }


// ---- ADD STUBS HERE FOR THE VIRTUALS THAT ARE DECLARED IN TelemetryModule.h ----
// Example (ONLY if TelemetryModule.h declares it exactly):
// void TelemetryModule::setup() {}
// void TelemetryModule::update(uint32_t nowMs) {}
// void TelemetryModule::handleEvent(const Event& evt) { (void)evt; }
// -----------------------------------------------------------------------------


// -----------------------------------------------------------------------------
// MotionController stubs
// -----------------------------------------------------------------------------
MotionController::MotionController(DcMotorManager& motors,
                                   uint8_t leftMotorId,
                                   uint8_t rightMotorId,
                                   float wheelBase,
                                   float maxLinear,
                                   float maxAngular,
                                   ServoManager* servoMgr,
                                   StepperManager* stepperMgr)
    : motors_(motors)
    , servoMgr_(servoMgr)
    , stepperMgr_(stepperMgr)
    , leftId_(leftMotorId)
    , rightId_(rightMotorId)
    , wheelBase_(wheelBase)
    , maxLinear_(maxLinear)
    , maxAngular_(maxAngular)
{
    for (uint8_t i = 0; i < ESP_MAX_SERVOS; ++i) {
        servoCurrent_[i] = 0.0f;
        servoStart_[i] = 0.0f;
        servoTarget_[i] = 0.0f;
        servoStartMs_[i] = 0;
        servoDurationMs_[i] = 0;
        servoActive_[i] = false;
    }
}

void MotionController::setVelocity(float vx, float omega) {
    vxRef_ = vx;
    omegaRef_ = omega;
}

void MotionController::stop() {
    vxRef_ = 0.0f;
    omegaRef_ = 0.0f;
}

float MotionController::vx() const { return vxRef_; }
float MotionController::omega() const { return omegaRef_; }

void MotionController::setAccelLimits(float maxLinAccel, float maxAngAccel) {
    maxLinAccel_ = maxLinAccel;
    maxAngAccel_ = maxAngAccel;
}

void MotionController::setBaseEnabled(bool enabled) {
    baseEnabled_ = enabled;
}

void MotionController::setServoTarget(uint8_t, float, uint32_t) {}
void MotionController::setServoImmediate(uint8_t, float) {}

void MotionController::moveStepperRelative(int, int, float) {}
void MotionController::enableStepper(int, bool) {}

void MotionController::update(float) {}
