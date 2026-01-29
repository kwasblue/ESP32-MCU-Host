#pragma once
#include "core/IController.h"

class PidController : public IController {
public:
  const char* type() const override { return "PID"; }

  void reset() override {
    integ_ = 0.0f;
    prev_err_ = 0.0f;
    prev_meas_ = 0.0f;
    first_ = true;
  }

  bool setParam(const char* key, float value) override {
    if (!key) return false;
    if (strcmp(key, "kp")==0) { kp_=value; return true; }
    if (strcmp(key, "ki")==0) { ki_=value; return true; }
    if (strcmp(key, "kd")==0) { kd_=value; return true; }
    if (strcmp(key, "out_min")==0) { out_min_=value; return true; }
    if (strcmp(key, "out_max")==0) { out_max_=value; return true; }
    if (strcmp(key, "i_min")==0) { i_min_=value; return true; }
    if (strcmp(key, "i_max")==0) { i_max_=value; return true; }
    return false;
  }

  bool step(SignalBus& sig, const ControlCtx& ctx) override {
    float ref=0, meas=0;
    if (!sig.get(io_.ref_id, ref)) return false;
    if (!sig.get(io_.meas_id, meas)) return false;

    float err = ref - meas;

    // dt safety for ms-based loops
    float dt = ctx.dt_s;
    if (dt < 1e-4f) dt = 1e-4f;
    if (dt > 0.2f)  dt = 0.2f;

    // Integral
    integ_ += err * dt;
    if (integ_ < i_min_) integ_ = i_min_;
    if (integ_ > i_max_) integ_ = i_max_;

    // Derivative (on measurement is often less noisy)
    float deriv = 0.0f;
    if (!first_) deriv = -(meas - prev_meas_) / dt;
    first_ = false;

    float u = (kp_ * err) + (ki_ * integ_) + (kd_ * deriv);
    if (u < out_min_) u = out_min_;
    if (u > out_max_) u = out_max_;

    sig.set(io_.out_id, u, ctx.now_ms);

    prev_err_ = err;
    prev_meas_ = meas;
    return true;
  }

private:
  float kp_=0.0f, ki_=0.0f, kd_=0.0f;
  float out_min_=-1.0f, out_max_=1.0f;
  float i_min_=-1.0f, i_max_=1.0f;

  float integ_=0.0f;
  float prev_err_=0.0f;
  float prev_meas_=0.0f;
  bool first_=true;
};
