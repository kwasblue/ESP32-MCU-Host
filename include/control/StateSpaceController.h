#pragma once
#include "control/IController.h"

// Minimal single-output SS controller: u = -Kx + Nr
// You can expand to Nx/Nu later.
class StateSpaceController : public IController {
public:
  const char* type() const override { return "SS"; }
  void reset() override { x_hat_=0.0f; }

  bool setParam(const char* key, float value) override {
    if (strcmp(key, "K")==0) { K_=value; return true; }
    if (strcmp(key, "N")==0) { N_=value; return true; }
    if (strcmp(key, "A")==0) { A_=value; return true; }
    if (strcmp(key, "B")==0) { B_=value; return true; }
    if (strcmp(key, "L")==0) { L_=value; return true; }
    return false;
  }

  bool step(SignalBus& sig, const ControlCtx& ctx) override {
    float r=0, y=0;
    if (!sig.get(io_.ref_id, r)) return false;
    if (!sig.get(io_.meas_id, y)) return false;

    // toy observer: x̂[k+1] = A x̂ + B u + L (y - x̂)
    // (for now assume C=1)
    float dt = ctx.dt_s;
    if (dt < 1e-4f) dt = 1e-4f;

    float u = (-K_ * x_hat_) + (N_ * r);
    float innov = (y - x_hat_);
    x_hat_ = (A_ * x_hat_) + (B_ * u) + (L_ * innov);

    sig.set(io_.out_id, u, ctx.now_ms);
    return true;
  }

private:
  float A_=1.0f, B_=0.0f, K_=0.0f, N_=1.0f, L_=0.0f;
  float x_hat_=0.0f;
};
