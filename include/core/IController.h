#pragma once
#include <Arduino.h>
#include "core/SignalBus.h"

struct ControlIO {
  // Minimal: one ref + one measurement + one output (expand later)
  uint16_t ref_id = 0;
  uint16_t meas_id = 0;
  uint16_t out_id = 0;
};

struct ControlCtx {
  uint32_t now_ms = 0;
  float dt_s = 0.0f;
};

class IController {
public:
  virtual ~IController() = default;

  virtual const char* type() const = 0;
  virtual void reset() = 0;

  // Configure which signals it uses
  virtual void configureIO(const ControlIO& io) { io_ = io; }
  virtual const ControlIO& io() const { return io_; }

  // Optional parameter update via JSON-ish values (you’ll call these from CommandHandler)
  virtual bool setParam(const char* key, float value) = 0;

  // Run one control step: read SignalBus, write output signal
  virtual bool step(SignalBus& sig, const ControlCtx& ctx) = 0;

protected:
  ControlIO io_;
};
