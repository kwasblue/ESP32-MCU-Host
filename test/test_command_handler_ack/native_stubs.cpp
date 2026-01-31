// test/test_command_handler_ack/native_stubs.cpp
// Stubs for native testing of CommandHandler ACK logic
// These stubs allow the test to link without pulling in the full firmware

#include "command/ModeManager.h"
#include "control/ControlKernel.h"
#include "control/Observer.h"
#include "control/SignalBus.h"
#include "core/LoopRates.h"
#include "module/LoggingModule.h"
#include "sensor/EncoderManager.h"
#include "module/TelemetryModule.h"
#include "motor/MotionController.h"

// ============================================================================
// LoopRates
// ============================================================================
static LoopRates g_loopRates{100, 50, 10};

LoopRates& getLoopRates() {
    return g_loopRates;
}

// ============================================================================
// ModeManager stubs
// ============================================================================
const char* robotModeToString(RobotMode mode) {
    switch (mode) {
        case RobotMode::BOOT:         return "BOOT";
        case RobotMode::DISCONNECTED: return "DISCONNECTED";
        case RobotMode::IDLE:         return "IDLE";
        case RobotMode::ARMED:        return "ARMED";
        case RobotMode::ACTIVE:       return "ACTIVE";
        case RobotMode::ESTOPPED:     return "ESTOPPED";
        default:                      return "UNKNOWN";
    }
}

void ModeManager::onHostHeartbeat(uint32_t) {}
void ModeManager::onMotionCommand(uint32_t) {}

bool ModeManager::validateVelocity(float vx, float omega, float& outVx, float& outOmega) {
    outVx = vx;
    outOmega = omega;
    return true;
}

void ModeManager::arm() { mode_ = RobotMode::ARMED; }
void ModeManager::activate() { mode_ = RobotMode::ACTIVE; }
void ModeManager::deactivate() { mode_ = RobotMode::ARMED; }
void ModeManager::disarm() { mode_ = RobotMode::IDLE; }
void ModeManager::estop() { mode_ = RobotMode::ESTOPPED; }
bool ModeManager::clearEstop() { mode_ = RobotMode::IDLE; return true; }

// ============================================================================
// SignalBus stubs
// ============================================================================
int SignalBus::indexOf_(uint16_t id) const {
    for (size_t i = 0; i < signals_.size(); ++i) {
        if (signals_[i].id == id) return static_cast<int>(i);
    }
    return -1;
}

bool SignalBus::define(uint16_t id, const char* name, Kind kind, float initial) {
    // Idempotent: update if exists
    for (auto& s : signals_) {
        if (s.id == id) {
            s.kind = kind;
            s.value = initial;
            s.ts_ms = 0;
            if (name && name[0] != '\0') {
                strncpy(s.name, name, NAME_MAX_LEN);
                s.name[NAME_MAX_LEN] = '\0';
            }
            return true;
        }
    }
    SignalDef def;
    def.id = id;
    def.kind = kind;
    def.value = initial;
    def.ts_ms = 0;
    if (name) {
        strncpy(def.name, name, NAME_MAX_LEN);
        def.name[NAME_MAX_LEN] = '\0';
    } else {
        def.name[0] = '\0';
    }
    signals_.push_back(def);
    return true;
}

bool SignalBus::exists(uint16_t id) const {
    return indexOf_(id) >= 0;
}

bool SignalBus::set(uint16_t id, float v, uint32_t now_ms) {
    int idx = indexOf_(id);
    if (idx < 0) return false;
    signals_[idx].value = v;
    signals_[idx].ts_ms = now_ms;
    return true;
}

bool SignalBus::get(uint16_t id, float& out) const {
    int idx = indexOf_(id);
    if (idx < 0) return false;
    out = signals_[idx].value;
    return true;
}

bool SignalBus::getTimestamp(uint16_t id, uint32_t& out) const {
    int idx = indexOf_(id);
    if (idx < 0) return false;
    out = signals_[idx].ts_ms;
    return true;
}

bool SignalBus::remove(uint16_t id) {
    int idx = indexOf_(id);
    if (idx < 0) return false;
    signals_.erase(signals_.begin() + idx);
    return true;
}

const SignalBus::SignalDef* SignalBus::find(uint16_t id) const {
    int idx = indexOf_(id);
    if (idx < 0) return nullptr;
    return &signals_[idx];
}

// ============================================================================
// ControlKernel stubs
// ============================================================================
bool ControlKernel::configureSlot(const SlotConfig& cfg, const char* type) {
    (void)cfg; (void)type;
    return true;
}

bool ControlKernel::enableSlot(uint8_t slot, bool enable) {
    (void)slot; (void)enable;
    return true;
}

bool ControlKernel::resetSlot(uint8_t slot) {
    (void)slot;
    return true;
}

bool ControlKernel::setParam(uint8_t slot, const char* key, float value) {
    (void)slot; (void)key; (void)value;
    return true;
}

bool ControlKernel::getParam(uint8_t slot, const char* key, float& value) const {
    (void)slot; (void)key;
    value = 0.0f;
    return true;
}

bool ControlKernel::setParamArray(uint8_t slot, const char* key, const float* values, size_t len) {
    (void)slot; (void)key; (void)values; (void)len;
    return true;
}

SlotConfig ControlKernel::getConfig(uint8_t slot) const {
    (void)slot;
    SlotConfig cfg{};
    return cfg;
}

SlotStatus ControlKernel::getStatus(uint8_t slot) const {
    (void)slot;
    SlotStatus st{};
    st.ok = true;
    return st;
}

void ControlKernel::step(uint32_t now_ms, float dt_s, SignalBus& signals, bool is_armed, bool is_active) {
    (void)now_ms; (void)dt_s; (void)signals; (void)is_armed; (void)is_active;
}

void ControlKernel::resetAll() {}
void ControlKernel::disableAll() {}

// ============================================================================
// LuenbergerObserver stubs
// ============================================================================
void LuenbergerObserver::configure(uint8_t num_states, uint8_t num_inputs, uint8_t num_outputs) {
    num_states_ = num_states;
    num_inputs_ = num_inputs;
    num_outputs_ = num_outputs;
    initialized_ = true;
}

void LuenbergerObserver::update(const float* u, const float* y, float dt, float* x_hat_out) {
    (void)u; (void)y; (void)dt;
    if (x_hat_out) {
        for (uint8_t i = 0; i < num_states_; ++i) {
            x_hat_out[i] = x_hat_[i];
        }
    }
}

void LuenbergerObserver::reset() {
    for (uint8_t i = 0; i < MAX_STATES; ++i) x_hat_[i] = 0.0f;
}

void LuenbergerObserver::initState(const float* x0) {
    if (x0) {
        for (uint8_t i = 0; i < num_states_; ++i) x_hat_[i] = x0[i];
    }
}

void LuenbergerObserver::setState(uint8_t idx, float value) {
    if (idx < MAX_STATES) x_hat_[idx] = value;
}

float LuenbergerObserver::getState(uint8_t idx) const {
    if (idx < MAX_STATES) return x_hat_[idx];
    return 0.0f;
}

bool LuenbergerObserver::setA(const float* A, size_t len) { (void)A; (void)len; return true; }
bool LuenbergerObserver::setB(const float* B, size_t len) { (void)B; (void)len; return true; }
bool LuenbergerObserver::setC(const float* C, size_t len) { (void)C; (void)len; return true; }
bool LuenbergerObserver::setL(const float* L, size_t len) { (void)L; (void)len; return true; }

bool LuenbergerObserver::setParam(const char* key, float value) {
    (void)key; (void)value;
    return true;
}

bool LuenbergerObserver::setParamArray(const char* key, const float* values, size_t len) {
    (void)key; (void)values; (void)len;
    return true;
}

// ============================================================================
// ObserverManager stubs
// ============================================================================
bool ObserverManager::configure(uint8_t slot, const ObserverConfig& config, uint16_t rate_hz) {
    if (slot >= MAX_SLOTS) return false;
    slots_[slot].config = config;
    slots_[slot].rate_hz = rate_hz;
    slots_[slot].configured = true;
    slots_[slot].observer.configure(config.num_states, config.num_inputs, config.num_outputs);
    return true;
}

bool ObserverManager::enable(uint8_t slot, bool en) {
    if (slot >= MAX_SLOTS || !slots_[slot].configured) return false;
    slots_[slot].enabled = en;
    return true;
}

bool ObserverManager::reset(uint8_t slot) {
    if (slot >= MAX_SLOTS) return false;
    slots_[slot].observer.reset();
    return true;
}

bool ObserverManager::setParam(uint8_t slot, const char* key, float value) {
    if (slot >= MAX_SLOTS) return false;
    return slots_[slot].observer.setParam(key, value);
}

bool ObserverManager::setParamArray(uint8_t slot, const char* key, const float* values, size_t len) {
    if (slot >= MAX_SLOTS) return false;
    return slots_[slot].observer.setParamArray(key, values, len);
}

void ObserverManager::step(uint32_t now_ms, float dt_s, SignalBus& signals) {
    (void)now_ms; (void)dt_s; (void)signals;
}

LuenbergerObserver* ObserverManager::getObserver(uint8_t slot) {
    if (slot >= MAX_SLOTS) return nullptr;
    return &slots_[slot].observer;
}

const ObserverManager::Slot& ObserverManager::getSlot(uint8_t slot) const {
    static Slot empty{};
    if (slot >= MAX_SLOTS) return empty;
    return slots_[slot];
}

void ObserverManager::resetAll() {
    for (auto& s : slots_) {
        s.observer.reset();
    }
}

void ObserverManager::disableAll() {
    for (auto& s : slots_) {
        s.enabled = false;
    }
}

// ============================================================================
// LoggingModule stub
// ============================================================================
LoggingModule* LoggingModule::s_instance = nullptr;

void LoggingModule::setLogLevel(const char* level) {
    (void)level;
}

// ============================================================================
// EncoderManager stubs - REMOVED: Now provided by header when HAS_ENCODER=0
// ============================================================================

// ============================================================================
// TelemetryModule stubs
// ============================================================================
TelemetryModule::TelemetryModule(EventBus& bus)
    : bus_(bus), intervalMs_(100), binaryEnabled_(true), jsonEnabled_(false) {}

void TelemetryModule::setup() {}
void TelemetryModule::loop(uint32_t now_ms) { (void)now_ms; }

void TelemetryModule::registerProvider(const char* name, JsonProviderFn fn) {
    (void)name; (void)fn;
}

void TelemetryModule::registerBinProvider(uint8_t section_id, BinProviderFn fn) {
    (void)section_id; (void)fn;
}

void TelemetryModule::setInterval(uint32_t intervalMs) {
    intervalMs_ = intervalMs;
}

void TelemetryModule::setBinaryEnabled(bool en) { binaryEnabled_ = en; }
void TelemetryModule::setJsonEnabled(bool en) { jsonEnabled_ = en; }

// ============================================================================
// MotionController stubs
// ============================================================================
MotionController::MotionController(
    DcMotorManager& motors,
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
    , baseEnabled_(false)
{}

void MotionController::setVelocity(float vx, float omega) {
    vxRef_ = vx;
    omegaRef_ = omega;
}

void MotionController::stop() {
    vxRef_ = 0;
    omegaRef_ = 0;
}

float MotionController::vx() const { return vxRef_; }
float MotionController::omega() const { return omegaRef_; }

void MotionController::setAccelLimits(float maxLinAccel, float maxAngAccel) {
    (void)maxLinAccel; (void)maxAngAccel;
}

void MotionController::setBaseEnabled(bool enabled) {
    baseEnabled_ = enabled;
}

void MotionController::setServoTarget(uint8_t servoId, float angleDeg, uint32_t durationMs) {
    (void)servoId; (void)angleDeg; (void)durationMs;
}

void MotionController::setServoImmediate(uint8_t servoId, float angleDeg) {
    (void)servoId; (void)angleDeg;
}

void MotionController::moveStepperRelative(int motorId, int steps, float speedStepsPerSec) {
    (void)motorId; (void)steps; (void)speedStepsPerSec;
}

void MotionController::enableStepper(int motorId, bool enabled) {
    (void)motorId; (void)enabled;
}

void MotionController::update(float dt) {
    (void)dt;
}