# ESP32 MCU Host Architecture

## Overview

The firmware follows a layered, event-driven architecture with compile-time modularity via feature flags.

## Layer Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                         main.cpp                                 │
│                    (initialization, loop)                        │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐              │
│  │  Transport  │  │   Command   │  │   Module    │              │
│  │   Layer     │  │    Layer    │  │    Layer    │              │
│  ├─────────────┤  ├─────────────┤  ├─────────────┤              │
│  │ UartTrans   │  │ CmdHandler  │  │ Telemetry   │              │
│  │ WifiTrans   │  │ MsgRouter   │  │ Heartbeat   │              │
│  │ BleTrans    │  │ ModeManager │  │ Identity    │              │
│  │ MqttTrans   │  │             │  │ Logging     │              │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘              │
│         │                │                │                      │
│         └────────────────┼────────────────┘                      │
│                          │                                       │
│  ┌───────────────────────┴───────────────────────┐              │
│  │                    EventBus                    │              │
│  │             (publish/subscribe)                │              │
│  └───────────────────────┬───────────────────────┘              │
│                          │                                       │
│  ┌───────────────────────┴───────────────────────┐              │
│  │                  Control Layer                 │              │
│  ├─────────────┬─────────────┬─────────────┬─────┤              │
│  │ SignalBus   │ CtrlKernel  │  Observer   │ PID │              │
│  └─────────────┴─────────────┴─────────────┴─────┘              │
│                          │                                       │
│  ┌───────────────────────┴───────────────────────┐              │
│  │               Hardware Abstraction             │              │
│  ├─────────────┬─────────────┬─────────────┬─────┤              │
│  │ DcMotor     │ Servo       │ IMU         │ Enc │              │
│  │ Manager     │ Manager     │ Manager     │ Mgr │              │
│  └─────────────┴─────────────┴─────────────┴─────┘              │
│                          │                                       │
│  ┌───────────────────────┴───────────────────────┐              │
│  │                ESP32 Hardware                  │              │
│  │          (GPIO, PWM, I2C, SPI, UART)           │              │
│  └───────────────────────────────────────────────┘              │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

## Core Components

### EventBus

Central publish/subscribe system for decoupled communication:

```cpp
// Event types
enum class EventType : uint8_t {
    HEARTBEAT,
    JSON_MESSAGE_RX,
    JSON_MESSAGE_TX,
    TELEMETRY_SYSTEM,
    TELEMETRY_MOTION,
    ESTOP_TRIGGERED,
    MODE_CHANGED,
};

// Usage
g_bus.subscribe([](const Event& evt) {
    if (evt.type == EventType::ESTOP_TRIGGERED) {
        // Handle ESTOP
    }
});

g_bus.publish(Event{EventType::HEARTBEAT, millis(), {}});
```

### SignalBus

Signal routing for control systems with O(1) lookup:

```cpp
SignalBus bus;

// Define signals
bus.define(SIG_VX_REF, "vx_ref", Kind::REF, 0.0f);
bus.define(SIG_VX_MEAS, "vx_meas", Kind::MEAS, 0.0f);

// High-rate access
bus.set(SIG_VX_REF, 0.5f, millis());

float vx;
bus.get(SIG_VX_MEAS, vx);

// Name-based access (slower, for configuration)
bus.setByName("vx_ref", 0.5f, millis());
```

### ModeManager

Robot state machine:

```
                    ┌─────────────┐
                    │   IDLE      │
                    │ (disarmed)  │
                    └──────┬──────┘
                           │ CMD_ARM
                           ▼
                    ┌─────────────┐
                    │   ARMED     │
                    │  (standby)  │
                    └──────┬──────┘
                           │ CMD_ACTIVATE
                           ▼
                    ┌─────────────┐
                    │   ACTIVE    │◄───┐
                    │ (motors on) │    │ CMD_CLEAR_ESTOP
                    └──────┬──────┘    │
                           │ ESTOP     │
                           ▼           │
                    ┌─────────────┐    │
                    │   ESTOP     │────┘
                    │ (emergency) │
                    └─────────────┘
```

## Communication Protocol

### Frame Format

```
┌────────┬────────┬────────┬──────────┬─────────────┬──────────┐
│ HEADER │ LEN_HI │ LEN_LO │ MSG_TYPE │   PAYLOAD   │ CHECKSUM │
│  0xAA  │   1B   │   1B   │    1B    │   N bytes   │    1B    │
└────────┴────────┴────────┴──────────┴─────────────┴──────────┘
```

### Message Flow

```
                Host                         MCU
                 │                            │
                 │    VERSION_REQUEST         │
                 ├───────────────────────────►│
                 │                            │
                 │    VERSION_RESPONSE        │
                 │◄───────────────────────────┤
                 │    (protocol, firmware)    │
                 │                            │
                 │    CMD_ARM {"wantAck":1}   │
                 ├───────────────────────────►│
                 │                            │
                 │    ACK {"ok":true}         │
                 │◄───────────────────────────┤
                 │                            │
                 │    HEARTBEAT               │
                 ├───────────────────────────►│
                 │                            │
                 │    TELEMETRY_BIN           │
                 │◄───────────────────────────┤
                 │                            │
```

## Control Loop Timing

```cpp
// Loop rates (configurable)
static constexpr uint32_t SAFETY_PERIOD_MS   = 10;   // 100 Hz
static constexpr uint32_t CONTROL_PERIOD_MS  = 10;   // 100 Hz
static constexpr uint32_t TELEMETRY_PERIOD_MS = 20;  // 50 Hz
static constexpr uint32_t HEARTBEAT_PERIOD_MS = 200; // 5 Hz
```

## Feature Flag System

Compile-time modularity using preprocessor:

```cpp
// FeatureFlags.h
#define HAS_DC_MOTOR  1
#define HAS_IMU       1
#define HAS_LIDAR     0

// Usage in code
#if HAS_DC_MOTOR
    g_dcMotorManager.begin();
    g_dcMotorManager.loop();
#endif
```

Disabled features compile to empty stubs (zero overhead).

## Memory Optimization

### EventPayload (optimized with variant)

```cpp
// Before: All fields always allocated
struct EventPayload {
    std::string json;           // 32+ bytes
    std::vector<uint8_t> bin;   // 24+ bytes
    TelemetryPayloads...        // 60+ bytes
};  // Total: ~120+ bytes always

// After: Only active payload allocated
using EventPayload = std::variant<
    std::monostate,
    std::string,
    std::vector<uint8_t>,
    TelemetrySystemPayload,
    TelemetryMotionPayload
>;  // Total: max(types) + 8 bytes
```

### SignalBus Lookup (O(1))

```cpp
// Lookup map maintained alongside vector
std::vector<SignalDef> signals_;
std::unordered_map<uint16_t, size_t> idToIndex_;

int indexOf_(uint16_t id) const {
    auto it = idToIndex_.find(id);
    return (it != idToIndex_.end()) ? it->second : -1;
}
```

## Extending the Firmware

### Adding a New Sensor

1. Create header `include/sensor/MySensor.h`:
```cpp
#pragma once
#include "config/FeatureFlags.h"

#if HAS_MY_SENSOR

class MySensorManager {
public:
    void begin();
    void loop();
    float getValue() const;
private:
    float value_ = 0.0f;
};

#else
class MySensorManager {
public:
    void begin() {}
    void loop() {}
    float getValue() const { return 0.0f; }
};
#endif
```

2. Create implementation `src/sensor/MySensor.cpp`

3. Add feature flag to `FeatureFlags.h`:
```cpp
#define HAS_MY_SENSOR 1
```

4. Instantiate in `main.cpp`:
```cpp
#if HAS_MY_SENSOR
MySensorManager g_mySensor;
#endif

void setup() {
    #if HAS_MY_SENSOR
    g_mySensor.begin();
    #endif
}

void loop() {
    #if HAS_MY_SENSOR
    g_mySensor.loop();
    #endif
}
```

### Adding a New Command

1. Add to CommandHandler.cpp:
```cpp
void CommandHandler::handleCommand(const JsonDocument& doc) {
    String cmd = doc["type"];

    if (cmd == "CMD_MY_COMMAND") {
        float param = doc["param"];
        // Handle command
        sendAck("CMD_MY_COMMAND", true);
        return;
    }
}
```

2. Document in `include/docs/commandSets.md`

## Testing

Native tests run on host (not ESP32):

```bash
# All tests
pio test -e native

# Specific test
pio test -e native -f test_signal_bus

# Verbose
pio test -e native -v
```

Test files in `test/`:
- `test_protocol/` - Frame encoding/decoding
- `test_event_bus/` - Pub/sub system
- `test_safety/` - Safety manager
- `test_command_handler_ack/` - Command handling
