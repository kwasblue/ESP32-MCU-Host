// include/core/PID.h
#pragma once

class PID {
public:
    PID(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f)
        : kp_(kp), ki_(ki), kd_(kd),
          prevError_(0.0f), integral_(0.0f),
          outMin_(-1.0f), outMax_(1.0f) {}

    void setGains(float kp, float ki, float kd) {
        kp_ = kp; ki_ = ki; kd_ = kd;
    }

    void setOutputLimits(float minVal, float maxVal) {
        outMin_ = minVal;
        outMax_ = maxVal;
    }

    void reset() {
        prevError_ = 0.0f;
        integral_  = 0.0f;
    }

    float compute(float target, float current, float dt) {
        float error = target - current;
        integral_ += error * dt;
        float derivative = (dt > 0.0f) ? (error - prevError_) / dt : 0.0f;
        prevError_ = error;

        float out = kp_ * error + ki_ * integral_ + kd_ * derivative;

        if (out > outMax_) out = outMax_;
        if (out < outMin_) out = outMin_;
        return out;
    }

private:
    float kp_, ki_, kd_;
    float prevError_;
    float integral_;
    float outMin_, outMax_;
};
