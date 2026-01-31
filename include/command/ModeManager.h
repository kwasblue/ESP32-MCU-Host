// include/core/ModeManager.h

#pragma once
#include <cstdint>
#include <functional>

enum class RobotMode : uint8_t {
    BOOT,
    DISCONNECTED,
    IDLE,
    ARMED,
    ACTIVE,
    ESTOPPED
};

const char* robotModeToString(RobotMode m);

struct SafetyConfig {
    uint32_t host_timeout_ms = 3000;
    uint32_t motion_timeout_ms = 500;
    float max_linear_vel = 2.0f;
    float max_angular_vel = 3.14f;
    int estop_pin = -1;
    int bypass_pin = -1;
    int relay_pin = -1;
};

class ModeManager {
public:
    using StopCallback = std::function<void()>;
    
    ModeManager() = default;
    
    void configure(const SafetyConfig& cfg) { cfg_ = cfg; }
    void begin();
    void update(uint32_t now_ms);
    
    // Watchdog feeds
    void onHostHeartbeat(uint32_t now_ms);
    void onMotionCommand(uint32_t now_ms);
    
    // Transitions
    void arm();
    void activate();
    void deactivate();
    void disarm();
    void estop();
    bool clearEstop();
    
    // Queries
    RobotMode mode() const { return mode_; }
    bool canMove() const { return mode_ == RobotMode::ARMED || mode_ == RobotMode::ACTIVE; }
    bool isEstopped() const { return mode_ == RobotMode::ESTOPPED; }
    bool isConnected() const { return mode_ != RobotMode::BOOT && mode_ != RobotMode::DISCONNECTED; }
    bool isBypassed() const { return bypassed_; }
    uint32_t hostAgeMs(uint32_t now_ms) const { return now_ms - lastHostHeartbeat_; }
    uint32_t motionAgeMs(uint32_t now_ms) const { return now_ms - lastMotionCmd_; }
    
    // Validation
    bool validateVelocity(float vx, float omega, float& out_vx, float& out_omega);
    
    // Callback
    void onStop(StopCallback cb) { stopCallback_ = cb; }
    
private:
    SafetyConfig cfg_;
    RobotMode mode_ = RobotMode::BOOT;
    RobotMode lastLoggedMode_ = RobotMode::BOOT;

    
    uint32_t lastHostHeartbeat_ = 0;
    uint32_t lastMotionCmd_ = 0;
    bool hostEverSeen_ = false;
    bool bypassed_ = false;
    
    StopCallback stopCallback_;
    
    void triggerStop();
    void readHardwareInputs();
    bool canTransition(RobotMode from, RobotMode to);

};

// Helper to convert mode to string
// inline const char* robotModeToString(RobotMode m) {
//     switch (m) {
//         case RobotMode::BOOT: return "BOOT";
//         case RobotMode::DISCONNECTED: return "DISCONNECTED";
//         case RobotMode::IDLE: return "IDLE";
//         case RobotMode::ARMED: return "ARMED";
//         case RobotMode::ACTIVE: return "ACTIVE";
//         case RobotMode::ESTOPPED: return "ESTOPPED";
//         default: return "UNKNOWN";
//     }
// }       