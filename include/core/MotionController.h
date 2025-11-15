#pragma once

class MotionController {
public:
    void setVelocity(float vx, float omega) {
        vx_    = vx;
        omega_ = omega;
    }

    void stop() {
        vx_    = 0.0f;
        omega_ = 0.0f;
    }

    float vx() const { return vx_; }
    float omega() const { return omega_; }

private:
    float vx_    = 0.0f;
    float omega_ = 0.0f;
};
