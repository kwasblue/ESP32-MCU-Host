#pragma once

enum class RobotMode {
    BOOT,
    IDLE,
    ARMED,
    ACTIVE,
    ESTOP
};

class ModeManager {
public:
    ModeManager() : mode_(RobotMode::BOOT) {}

    void setMode(RobotMode m) { mode_ = m; }
    RobotMode mode() const { return mode_; }

    bool canMove() const {
        return mode_ == RobotMode::ARMED || mode_ == RobotMode::ACTIVE;
    }

private:
    RobotMode mode_;
};
