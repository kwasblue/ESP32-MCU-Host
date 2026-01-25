// test/common/NativeStubs.cpp
// Provides minimal native-only definitions for MCU-only code paths so unit tests can link.

#ifdef PIO_UNIT_TESTING

#include <cstdint>
#include <string>

// Include project headers using include/ as the root (PlatformIO adds include/ to the include path)
#include "core/ModeManager.h"
#include "modules/LoggingModule.h"
#include "modules/TelemetryModule.h"
#include "managers/EncoderManager.h"
#include "core/MotionController.h"

// -------------------- robotModeToString --------------------
// Declared somewhere in your project (ModeManager/State), but implementation lives in MCU build.
// Provide a native definition so CommandHandler can link.
const char* robotModeToString(RobotMode mode) {
    switch (mode) {
        case RobotMode::DISARMED:  return "DISARMED";
        case RobotMode::ARMED:     return "ARMED";
        case RobotMode::ACTIVE:    return "ACTIVE";
        case RobotMode::ESTOP:     return "ESTOP";
        default:                   return "UNKNOWN";
    }
}

// -------------------- ModeManager stubs --------------------
void ModeManager::arm() {}
void ModeManager::disarm() {}
void ModeManager::activate() {}
void ModeManager::deactivate() {}
void ModeManager::estop() {}
void ModeManager::clearEstop() {}

void ModeManager::onHostHeartbeat(uint32_t /*ms*/) {}
void ModeManager::onMotionCommand(uint32_t /*ms*/) {}

// If your real validateVelocity clamps/filters, this can be very simple for native tests:
bool ModeManager::validateVelocity(float vx, float omega, float& outVx, float& outOmega) {
    outVx = vx;
    outOmega = omega;
    return true;
}

// -------------------- LoggingModule stubs --------------------
// Your linker error mentions LoggingModule::s_instance and setLogLevel().
LoggingModule* LoggingModule::s_instance = nullptr;

void LoggingModule::setLogLevel(const char* /*level*/) {}

// -------------------- EncoderManager stubs --------------------
// Your linker error lists EncoderManager ctor, attach/reset/getCount.
EncoderManager::EncoderManager() {}

bool EncoderManager::attach(uint8_t /*id*/, uint8_t /*pinA*/, uint8_t /*pinB*/) {
    return true;
}

void EncoderManager::reset(uint8_t /*id*/) {}

int32_t EncoderManager::getCount(uint8_t /*id*/) const {
    return 0;
}

// -------------------- TelemetryModule stubs --------------------
TelemetryModule::TelemetryModule(EventBus& /*bus*/) {}
TelemetryModule::~TelemetryModule() = default;

void TelemetryModule::setInterval(uint32_t /*ms*/) {}

// If TelemetryModule overrides handleEvent out-of-line in the real build,
// define it here too so the vtable is complete.
void TelemetryModule::handleEvent(const Event& /*evt*/) {}

// -------------------- MotionController stubs --------------------
// Your linker error includes MotionController constructor + stop + setServoTarget + moveStepperRelative.
MotionController::MotionController(
    DcMotorManager& /*motors*/,
    uint8_t /*leftMotorId*/,
    uint8_t /*rightMotorId*/,
    float /*wheelBase*/,
    float /*maxLinear*/,
    float /*maxAngular*/,
    ServoManager* /*servoMgr*/,
    StepperManager* /*stepperMgr*/
) {}

void MotionController::setVelocity(float /*vx*/, float /*omega*/) {}
void MotionController::stop() {}

void MotionController::setServoTarget(uint8_t /*servoId*/, float /*angleDeg*/, uint32_t /*durationMs*/) {}
void MotionController::moveStepperRelative(int /*motorId*/, int /*steps*/, float /*speedStepsPerSec*/) {}

#endif // PIO_UNIT_TESTING
