# MARA Firmware Architecture

**Modular Asynchronous Robotics Architecture - ESP32 Firmware Component**

**Architecture Grade: A+** (Production-ready, highly extensible)

---

## Overview

The MARA Firmware follows a layered, real-time architecture with:
- **FreeRTOS control task** for deterministic motor control
- **Dependency injection** via ServiceContext
- **Result<T> error handling** for robust error propagation
- **Modular setup** via ISetupModule pattern
- **CRC16-CCITT** protocol integrity
- **Real-time contract enforcement** via RT_SAFE annotations
- **IntentBuffer** for command/actuation boundary separation
- **Capability gating** for runtime feature availability
- **Self-registration** for handlers, modules, sensors, transports, and actuators

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              EXECUTION MODEL                                 │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │           FreeRTOS Control Task (Core 1, 100Hz, Priority 5)          │    │
│  │  ┌─────────────────────────────────────────────────────────────┐    │    │
│  │  │  runControlLoop()                                            │    │    │
│  │  │  ├── Encoder velocity calculation                           │    │    │
│  │  │  ├── DC motor velocity PID                                   │    │    │
│  │  │  └── MotionController.update() ← SET_VEL commands            │    │    │
│  │  └─────────────────────────────────────────────────────────────┘    │    │
│  │  ┌─────────────────────────────────────────────────────────────┐    │    │
│  │  │  ControlModule.loop()                                        │    │    │
│  │  │  ├── Observers (state estimation)                            │    │    │
│  │  │  └── ControlKernel (PID/LQR slots) ← SET_SIGNAL commands     │    │    │
│  │  └─────────────────────────────────────────────────────────────┘    │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                                                              │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                    Main Loop (Core 0/1, Cooperative)                 │    │
│  │  ┌───────────────┐ ┌───────────────┐ ┌───────────────┐              │    │
│  │  │ Safety Loop   │ │ Telemetry     │ │ Host/Comms    │              │    │
│  │  │ (100Hz)       │ │ (10-50Hz)     │ │ (best effort) │              │    │
│  │  │ ├─ModeManager │ │ └─Send data   │ │ ├─Cmd parsing │              │    │
│  │  │ ├─Watchdogs   │ │               │ │ ├─WiFi/BLE    │              │    │
│  │  │ └─E-stop      │ │               │ │ └─OTA         │              │    │
│  │  └───────────────┘ └───────────────┘ └───────────────┘              │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Layer Diagram

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              Application Layer                               │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                         main.cpp + Setup Modules                     │    │
│  │  SetupWifi │ SetupOta │ SetupSafety │ SetupMotors │ SetupTelemetry  │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
├─────────────────────────────────────────────────────────────────────────────┤
│                              Service Layer                                   │
│  ┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐          │
│  │  ServiceContext  │  │  ServiceStorage  │  │    Result<T>     │          │
│  │  (DI container)  │  │  (owns instances)│  │  (error handling)│          │
│  └──────────────────┘  └──────────────────┘  └──────────────────┘          │
├─────────────────────────────────────────────────────────────────────────────┤
│                              Command Layer                                   │
│  ┌────────────┐ ┌────────────┐ ┌────────────┐ ┌────────────┐               │
│  │ MsgRouter  │ │ CmdRegistry│ │  Domain    │ │ ModeManager│               │
│  │ (framing)  │ │ (dispatch) │ │  Handlers  │ │ (state FSM)│               │
│  └────────────┘ └────────────┘ └────────────┘ └────────────┘               │
├─────────────────────────────────────────────────────────────────────────────┤
│                              Control Layer                                   │
│  ┌────────────┐ ┌────────────┐ ┌────────────┐ ┌────────────┐               │
│  │ SignalBus  │ │CtrlKernel  │ │ Observer   │ │  Motion    │               │
│  │ (O(1) sigs)│ │ (PID/LQR)  │ │ (state est)│ │ Controller │               │
│  └────────────┘ └────────────┘ └────────────┘ └────────────┘               │
├─────────────────────────────────────────────────────────────────────────────┤
│                              Hardware Layer                                  │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐          │
│  │ DcMotor  │ │ Stepper  │ │  Servo   │ │   IMU    │ │ Encoder  │          │
│  │ Manager  │ │ Manager  │ │ Manager  │ │ Manager  │ │ Manager  │          │
│  └──────────┘ └──────────┘ └──────────┘ └──────────┘ └──────────┘          │
├─────────────────────────────────────────────────────────────────────────────┤
│                              Transport Layer                                 │
│  ┌────────────┐ ┌────────────┐ ┌────────────┐ ┌────────────┐               │
│  │   UART     │ │   WiFi     │ │    BLE     │ │   MQTT     │               │
│  │ Transport  │ │ Transport  │ │ Transport  │ │ Transport  │               │
│  └────────────┘ └────────────┘ └────────────┘ └────────────┘               │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Safety Architecture

### State Machine

```
        ┌──────────────────────────────────────────────────────────┐
        │                                                          │
        ▼                                                          │
┌──────────────┐     ┌──────────────┐     ┌──────────────┐        │
│     BOOT     │────►│ DISCONNECTED │────►│     IDLE     │        │
└──────────────┘     └──────────────┘     └──────┬───────┘        │
                                                  │ ARM            │
                                                  ▼                │
                                          ┌──────────────┐        │
                                          │    ARMED     │◄───────┤
                                          └──────┬───────┘        │
                                                  │ ACTIVATE      │
                                                  ▼                │
                                          ┌──────────────┐        │
                            ┌────────────►│    ACTIVE    │        │
                            │             └──────┬───────┘        │
                            │                    │ ESTOP          │
                            │ CLEAR_ESTOP        ▼                │
                            │             ┌──────────────┐        │
                            └─────────────│   ESTOPPED   │────────┘
                                          └──────────────┘
```

### Safety Features

| Feature | Implementation | Location |
|---------|---------------|----------|
| Critical sections | `portENTER_CRITICAL()` | ModeManager state transitions |
| SignalBus thread safety | `portENTER_CRITICAL()` spinlock | SignalBus get/set/snapshot |
| Hardware watchdog | 2 second timeout | ESP32 task watchdog |
| Host timeout | 2 second default | ModeManager.update() |
| Motion timeout | 2 second default | ModeManager.update() |
| Emergency stop | Direct motor disable | SetupSafety callback |
| Critical module halt | System stops on failure | main.cpp setup |

### Critical Modules

These modules are marked `isCritical() = true`. If they fail during setup, the system halts:

- **SetupSafety** - ModeManager must be operational
- **SetupTransport** - Must be able to receive commands

## Real-Time Contract Enforcement

### RT_SAFE Annotations

Functions safe to call from the control loop are annotated with `RT_SAFE`:

```cpp
// include/core/RealTimeContract.h
#define RT_SAFE   [[nodiscard("RT_SAFE")]]  // Safe for control loop
#define RT_UNSAFE [[deprecated("RT_UNSAFE: not safe in control loop")]]
```

**Usage:**
```cpp
// Correct: O(1), no allocation, bounded time
RT_SAFE bool SignalBus::get(uint16_t id, float& out) const;
RT_SAFE bool IntentBuffer::consumeVelocityIntent(VelocityIntent& out);

// Wrong: May allocate, unbounded time
RT_UNSAFE void sendTelemetry();  // Uses JSON
```

### RT_ZONE Macros

Control loop code is wrapped with zone markers for static analysis:

```cpp
void runControlLoop() {
    RT_ZONE_BEGIN("ControlLoop");

    // All code here must be RT_SAFE:
    // - No heap allocation
    // - O(1) operations only
    // - Bounded execution time
    bus.get(REF_VX, ref_vx);
    intents.consumeVelocityIntent(vel);
    controller.step(ref_vx, meas_vx, dt);

    RT_ZONE_END();
}
```

### Forbidden in RT Zones

| Operation | Why Forbidden | Alternative |
|-----------|---------------|-------------|
| `new`/`delete` | Heap fragmentation, unbounded time | Pre-allocate at setup |
| `std::vector::push_back` | May reallocate | Fixed-size arrays |
| `std::string` operations | Heap-backed | `const char*` literals |
| `JsonDocument` | Allocates | Pre-parsed config |
| `Serial.print` | Unbounded I/O | Deferred logging |

## Command/Actuation Boundary (IntentBuffer)

### Purpose

The IntentBuffer decouples command handlers (async, variable rate) from the control loop (deterministic, fixed rate). Commands set "intents" which the control loop consumes.

```
Command → Handler → IntentBuffer → ControlLoop → Actuator
                                        ↓
                                   Telemetry (read-only)
```

### Intent Types

| Intent | Producer | Consumer | Semantics |
|--------|----------|----------|-----------|
| VelocityIntent | MotionHandler | LoopControl | Latest-wins (single slot) |
| ServoIntent | ServoHandler | LoopControl | Latest-wins per ID |
| DcMotorIntent | DcMotorHandler | LoopControl | Latest-wins per ID |
| StepperIntent | StepperHandler | LoopControl | Latest-wins per ID |
| SignalIntent | ControlHandler | LoopControl | Ring buffer (16 slots, FIFO) |

### Producer API (Command Handlers)

```cpp
// Called from command handlers - NOT RT_SAFE (spinlock-protected)
intents->setVelocityIntent(vx, omega, now_ms);
intents->setServoIntent(id, angle, duration_ms, now_ms);
intents->setDcMotorIntent(id, speed, now_ms);
intents->queueSignalIntent(signal_id, value, now_ms);
```

### Consumer API (Control Loop)

```cpp
// Called from control loop - RT_SAFE (spinlock < 10μs)
RT_SAFE bool consumeVelocityIntent(VelocityIntent& out);
RT_SAFE bool consumeServoIntent(uint8_t id, ServoIntent& out);
RT_SAFE bool consumeDcMotorIntent(uint8_t id, DcMotorIntent& out);
RT_SAFE bool consumeSignalIntent(SignalIntent& out);
```

### What Bypasses Intents

Direct actuator calls are allowed ONLY for:
- **Configuration**: attach/detach, PID gains, enable/disable
- **Safety overrides**: ESTOP (immediate motor stop)
- **Debug/diagnostic**: motor mapping queries

### Override Behavior

| Event | Action |
|-------|--------|
| ESTOP | `clearAll()` + immediate motor stop |
| STOP | `setVelocityIntent(0, 0, now)` |
| DISARM | `clearAll()` + transition to IDLE |
| DEACTIVATE | `clearAll()` + transition to ARMED |

## Capability Gating

### HandlerCap Bitmask

Handlers declare required capabilities. Disabled features skip handler dispatch:

```cpp
// include/command/IStringHandler.h
namespace HandlerCap {
    constexpr uint32_t NONE        = 0;
    constexpr uint32_t GPIO        = (1 << 0);
    constexpr uint32_t PWM         = (1 << 1);
    constexpr uint32_t SERVO       = (1 << 2);
    constexpr uint32_t DC_MOTOR    = (1 << 3);
    constexpr uint32_t STEPPER     = (1 << 4);
    constexpr uint32_t ENCODER     = (1 << 5);
    constexpr uint32_t IMU         = (1 << 6);
    constexpr uint32_t ULTRASONIC  = (1 << 7);
    constexpr uint32_t WIFI        = (1 << 8);
    constexpr uint32_t CONTROL     = (1 << 9);
}
```

### Handler Implementation

```cpp
class ServoHandler : public IStringHandler {
public:
    uint32_t requiredCaps() const override {
        return HandlerCap::SERVO;  // Requires HAS_SERVO=1
    }
    // ...
};
```

### Registry Filtering

```cpp
// HandlerRegistry checks caps before dispatch
void HandlerRegistry::setAvailableCaps(uint32_t caps) {
    availableCaps_ = caps;
}

bool HandlerRegistry::dispatch(const char* cmd, ...) {
    IStringHandler* h = findHandler(cmd);
    if (!h) return false;

    // Skip if required capabilities not available
    if ((h->requiredCaps() & availableCaps_) != h->requiredCaps()) {
        return false;  // Handler's features disabled
    }

    h->handle(cmd, payload, ctx);
    return true;
}
```

### Build-Time Caps

Capability mask is built from feature flags:

```cpp
// config/FeatureFlags.h
inline uint32_t buildCapabilityMask() {
    uint32_t caps = 0;
    #if HAS_SERVO
    caps |= HandlerCap::SERVO;
    #endif
    #if HAS_DC_MOTOR
    caps |= HandlerCap::DC_MOTOR;
    #endif
    // ...
    return caps;
}
```

## Unified Configuration (RobotConfig)

### Structure

All robot configuration consolidated in one struct:

```cpp
// include/config/RobotConfig.h
namespace config {
struct RobotConfig {
    // Safety thresholds
    struct Safety {
        uint16_t host_timeout_ms = 2000;
        uint16_t motion_timeout_ms = 2000;
        uint16_t heartbeat_interval_ms = 500;
    } safety;

    // Loop rates
    struct LoopRates {
        uint16_t control_hz = 100;
        uint16_t safety_hz = 100;
        uint16_t telemetry_hz = 10;
    } rates;

    // FreeRTOS control task
    struct ControlTask {
        uint16_t rate_hz = 100;
        uint16_t stack_size = 4096;
        uint8_t priority = 5;
        uint8_t core = 1;
    } control_task;

    // Network settings
    struct Network {
        uint16_t tcp_port = 8080;
        uint32_t uart_baud = 115200;
    } network;

    // Motion limits
    struct Motion {
        float max_linear_vel = 1.0f;
        float max_angular_vel = 3.14159f;
    } motion;

    // Runtime override from JSON
    bool applyOverrides(const char* json);
    int toJson(char* buffer, size_t bufferSize) const;
};
}
```

### Usage

```cpp
// In main.cpp
config::RobotConfig cfg;
cfg.applyOverrides(SD.readFile("/config.json"));

g_storage.initTransports(Serial, cfg.network.uart_baud, cfg.network.tcp_port);
```

## Communication Protocol

### Frame Format (CRC16-CCITT)

```
┌────────┬────────┬────────┬──────────┬─────────────┬──────────────┐
│ HEADER │ LEN_HI │ LEN_LO │ MSG_TYPE │   PAYLOAD   │    CRC16     │
│  0xAA  │   1B   │   1B   │    1B    │   N bytes   │     2B       │
└────────┴────────┴────────┴──────────┴─────────────┴──────────────┘

CRC16-CCITT: polynomial 0x1021, initial 0xFFFF
Calculated over: LEN_HI + LEN_LO + MSG_TYPE + PAYLOAD
```

### Binary vs JSON Commands

| Type | Use Case | Size | Latency |
|------|----------|------|---------|
| **Binary** | Real-time control (SET_VEL, SET_SIGNAL, HEARTBEAT) | 7-9 bytes | ~5ms |
| **JSON** | Configuration (ARM, PID gains, slot config) | 40-100 bytes | ~50ms |

### Binary Opcodes

```cpp
enum class Opcode : uint8_t {
    SET_VEL     = 0x10,  // vx(f32), omega(f32) - 9 bytes
    SET_SIGNAL  = 0x11,  // id(u16), value(f32) - 7 bytes
    SET_SIGNALS = 0x12,  // count(u8), [id(u16), value(f32)]*
    HEARTBEAT   = 0x20,  // no payload - 1 byte
    STOP        = 0x21,  // no payload - 1 byte
};
```

### Telemetry Section IDs

Binary telemetry uses section IDs defined in `include/telemetry/TelemetrySections.h` (auto-generated from Host's `platform_schema.py`):

```cpp
namespace TelemetrySections {
enum class SectionId : uint8_t {
    TELEM_IMU            = 0x01,  // IMU sensor data
    TELEM_ULTRASONIC     = 0x02,  // Ultrasonic distance
    TELEM_LIDAR          = 0x03,  // LiDAR distance
    TELEM_ENCODER0       = 0x04,  // Encoder 0 ticks
    TELEM_STEPPER0       = 0x05,  // Stepper motor 0 state
    TELEM_DC_MOTOR0      = 0x06,  // DC motor 0 state
    TELEM_CTRL_SIGNALS   = 0x10,  // Control signal bus values
    TELEM_CTRL_OBSERVERS = 0x11,  // Observer state estimates
    TELEM_CTRL_SLOTS     = 0x12,  // Control slot status
};
}
```

## Extensibility System

MARA uses a unified self-registration pattern across all component types. Adding new functionality requires **1 file + 1 include**.

### Extension Summary

| Component | Interface | Macro | Registry | Files to Add |
|-----------|-----------|-------|----------|--------------|
| Command Handler | `IStringHandler` | `REGISTER_HANDLER` | `HandlerRegistry` | 1+1 |
| Runtime Module | `IModule` | `REGISTER_MODULE` | `ModuleManager` | 1+1 |
| Sensor | `ISensor` | `REGISTER_SENSOR` | `SensorRegistry` | 1+1 |
| Transport | `IRegisteredTransport` | `REGISTER_TRANSPORT` | `TransportRegistry` | 1+1 |
| Actuator | `IActuator` | `REGISTER_ACTUATOR` | `ActuatorRegistry` | 1+1 |
| Setup Module | `ISetupModule` | (manifest) | `SetupManifest` | 1+2 |

### Self-Registration Pattern

All registries follow the same pattern:

```
┌─────────────────────────────────────────────────────────────────┐
│                  STATIC REGISTRATION                             │
│         (before main(), via REGISTER_* macros)                   │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  HandlerRegistry   ◄── REGISTER_HANDLER(MyHandler)              │
│  ModuleManager     ◄── REGISTER_MODULE(MyModule)                │
│  SensorRegistry    ◄── REGISTER_SENSOR(MySensor)                │
│  TransportRegistry ◄── REGISTER_TRANSPORT(MyTransport)          │
│  ActuatorRegistry  ◄── REGISTER_ACTUATOR(MyActuator)            │
│                                                                  │
├─────────────────────────────────────────────────────────────────┤
│                    RUNTIME INITIALIZATION                        │
│              (in setup(), via ServiceContext)                    │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  1. Set capability flags from FeatureFlags.h                    │
│  2. Call initAll() on each registry with ServiceContext         │
│  3. Components get dependencies via init(ServiceContext&)       │
│  4. Components skip if requiredCaps() not available             │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### Deferred Initialization Pattern

All self-registering components use deferred initialization:

```cpp
class MyComponent : public IActuator {  // or ISensor, IModule, etc.
public:
    MyComponent() = default;  // Default constructor (no dependencies)

    void init(ServiceContext& ctx) override {
        // Get dependencies from context AFTER construction
        gpio_ = ctx.gpio;
        pwm_ = ctx.pwm;
        online_ = (gpio_ && pwm_);
    }

    void setup() override {
        // Configure hardware using Pins:: constants
        attach(0, Pins::MOTOR_LEFT_IN1, ...);
    }
};

REGISTER_ACTUATOR(MyComponent);  // Static registration before main()
```

### Capability Gating

Each component declares required capabilities. Disabled features are skipped:

```cpp
uint32_t requiredCaps() const override {
    return ActuatorCap::DC_MOTOR;  // Skip if HAS_DC_MOTOR=0
}
```

Capability masks are built from feature flags at startup:
```cpp
uint32_t caps = 0;
#if HAS_DC_MOTOR
caps |= ActuatorCap::DC_MOTOR;
#endif
ActuatorRegistry::instance().setAvailableCaps(caps);
```

### Adding Components

#### Add a Sensor (1 file + 1 include)

```cpp
// include/sensor/TempSensor.h
#include "sensor/ISensor.h"
#include "sensor/SensorMacros.h"
#include "config/PinConfig.h"

class TempSensor : public mcu::ISensor {
public:
    static constexpr const char* NAME = "temp";
    const char* name() const override { return NAME; }
    uint32_t requiredCaps() const override { return mcu::SensorCap::TEMP; }

    void init() override {
        pinMode(Pins::TEMP_PIN, INPUT);
        online_ = true;
    }

    void loop(uint32_t now_ms) override { /* read sensor */ }
    void toJson(JsonObject& out) const override { out["c"] = celsius_; }
};
REGISTER_SENSOR(TempSensor);
```

Include in `AllSensors.h`:
```cpp
#include "sensor/TempSensor.h"
```

#### Add an Actuator (1 file + 1 include)

```cpp
// include/motor/BrushlessActuator.h
#include "motor/IActuator.h"
#include "motor/ActuatorRegistry.h"

class BrushlessActuator : public mcu::IActuator {
public:
    static constexpr const char* NAME = "brushless";
    const char* name() const override { return NAME; }

    void init(mcu::ServiceContext& ctx) override {
        pwm_ = ctx.pwm;
        online_ = (pwm_ != nullptr);
    }

    void setup() override {
        attach(0, Pins::BRUSHLESS_PWM);
    }

    void stopAll() override { /* stop all motors */ }
};
REGISTER_ACTUATOR(BrushlessActuator);
```

Include in `AllActuators.h`:
```cpp
#include "motor/BrushlessActuator.h"
```

#### Add a Transport (1 file + 1 include)

```cpp
// include/transport/CanTransport.h
#include "transport/TransportRegistry.h"

class CanTransport : public mcu::IRegisteredTransport {
public:
    static constexpr const char* NAME = "can";
    const char* name() const override { return NAME; }
    uint32_t requiredCaps() const override { return mcu::TransportCap::CAN; }

    void begin() override { /* init CAN */ }
    void loop() override { /* poll */ }
    bool sendBytes(const uint8_t* data, size_t len) override { return true; }
};
REGISTER_TRANSPORT(CanTransport);
```

### Registries in ServiceContext

All registries are accessible via ServiceContext:

```cpp
struct ServiceContext {
    // ...
    HandlerRegistry*   handlerRegistry   = nullptr;
    ModuleManager*     moduleManager     = nullptr;
    SensorRegistry*    sensorRegistry    = nullptr;
    TransportRegistry* transportRegistry = nullptr;
    ActuatorRegistry*  actuatorRegistry  = nullptr;
};
```

Access via context (preferred):
```cpp
ctx.sensorRegistry->find("temp");
ctx.actuatorRegistry->get<DcMotorActuator>();
```

## Command Dispatch Architecture

### Dual-Registry System

MARA uses two complementary handler systems:

| System | Interface | Registration | Use Case |
|--------|-----------|--------------|----------|
| **Legacy** | `ICommandHandler` | Manual in `ServiceStorage` | Existing handlers, enum-based |
| **New (Extensible)** | `IStringHandler` | Self-registration via macro | New handlers, string-based |

Commands are processed through a dual-dispatch architecture:

```
┌─────────────────────────────────────────────────────────────────────┐
│                         Command Flow                                 │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  Transport (WiFi/UART/BLE)                                          │
│       │                                                              │
│       ▼                                                              │
│  MessageRouter ──► EventBus.publish(JSON_MESSAGE_RX)                │
│                          │                                           │
│                          ▼                                           │
│                   CommandRegistry.handleEvent()                      │
│                          │                                           │
│                          ▼                                           │
│                   parseJsonToMessage() ──► typeStr + CmdType         │
│                          │                                           │
│               ┌──────────┴──────────┐                                │
│               ▼                     ▼                                │
│      HandlerRegistry         CommandRegistry                         │
│      (string-based)          (enum-based)                           │
│      try dispatch first      fallback if not                        │
│               │              handled                                │
│          ┌────┴────┐              │                                  │
│          ▼         ▼              ▼                                  │
│   NewHandler   LedHandler    SafetyHandler  MotionHandler ...       │
│   (self-reg)   (self-reg)    (legacy)       (legacy)                │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

### Adding New Handlers (Extensible System)

The new extensibility system reduces handler addition from **5 files to 1 file**:

**Before (Legacy):**
1. Create `handlers/NewHandler.h`
2. Edit `handlers/AllHandlers.h` - add include
3. Edit `config/CommandDefs.h` - add CmdType enum
4. Edit `core/ServiceStorage.h` - add member + init + destructor
5. Edit `core/ServiceContext.h` - add member + buildContext

**After (New System):**
1. Create handler file with `REGISTER_HANDLER` macro - done!

#### Example: Adding a New Handler

```cpp
// include/command/handlers/LedHandler.h
#pragma once
#include "command/IStringHandler.h"
#include "command/HandlerMacros.h"
#include "command/CommandContext.h"

class LedHandler : public IStringHandler {
public:
    // Commands this handler supports (null-terminated)
    static constexpr const char* CMDS[] = {
        "CMD_LED_ON",
        "CMD_LED_OFF",
        "CMD_LED_BLINK",  // No CmdType enum needed!
        nullptr
    };

    const char* const* commands() const override { return CMDS; }
    const char* name() const override { return "LedHandler"; }
    int priority() const override { return 100; }  // Lower = earlier

    void handle(const char* cmd, JsonVariantConst payload,
                CommandContext& ctx) override {
        if (strcmp(cmd, "CMD_LED_ON") == 0) {
            digitalWrite(LED_BUILTIN, HIGH);
            JsonDocument resp;
            ctx.sendAck(cmd, true, resp);
        }
        else if (strcmp(cmd, "CMD_LED_OFF") == 0) {
            digitalWrite(LED_BUILTIN, LOW);
            JsonDocument resp;
            ctx.sendAck(cmd, true, resp);
        }
        // ... handle other commands
    }
};

// Self-registration - that's it!
REGISTER_HANDLER(LedHandler);
```

Then include in `AllHandlers.h` (or any compiled file):
```cpp
#include "command/handlers/LedHandler.h"
```

### IStringHandler Interface

```cpp
class IStringHandler {
public:
    virtual ~IStringHandler() = default;

    // Null-terminated array of command strings
    virtual const char* const* commands() const = 0;

    // Handle command by string name
    virtual void handle(const char* cmd, JsonVariantConst payload,
                        CommandContext& ctx) = 0;

    // Handler name for debugging
    virtual const char* name() const = 0;

    // Priority (lower = earlier). Default 100. Safety handlers use < 50.
    virtual int priority() const { return 100; }
};
```

### HandlerRegistry

The `HandlerRegistry` singleton manages self-registered handlers:

```cpp
class HandlerRegistry {
public:
    static HandlerRegistry& instance();

    void registerHandler(IStringHandler* handler);  // Called by macro
    void finalize();  // Sort by priority, called in initCommands()

    bool dispatch(const char* cmd, JsonVariantConst payload,
                  CommandContext& ctx);
    IStringHandler* findHandler(const char* cmd);
    size_t handlerCount() const;
};
```

**Memory Impact:** ~250 bytes for 15 handlers (negligible on ESP32).

### ICommandHandler Interface (Legacy)

```cpp
class ICommandHandler {
public:
    virtual bool canHandle(CmdType cmd) const = 0;
    virtual void handle(CmdType cmd, JsonVariantConst payload,
                        CommandContext& ctx) = 0;
    virtual const char* name() const { return "UnnamedHandler"; }
};
```

### Domain Handlers

| Handler | Commands | Responsibility | System |
|---------|----------|----------------|--------|
| SafetyHandler | ARM, DISARM, ESTOP, CLEAR_ESTOP | Safety state transitions | Legacy |
| MotionHandler | SET_VEL, STOP | Differential drive control | Legacy |
| GpioHandler | GPIO_WRITE, GPIO_READ, GPIO_TOGGLE | Digital I/O | Legacy |
| ServoHandler | SERVO_ATTACH, SERVO_SET | Servo motor control | Legacy |
| StepperHandler | STEPPER_ENABLE, STEPPER_MOVE | Stepper motor control | Legacy |
| DcMotorHandler | DC_SET_SPEED, DC_VEL_PID_* | DC motor + velocity PID | Legacy |
| SensorHandler | ULTRASONIC_*, ENCODER_* | Sensor configuration | Legacy |
| TelemetryHandler | TELEM_SET_INTERVAL | Telemetry settings | Legacy |
| ControlHandler | CTRL_SLOT_*, CTRL_SIGNAL_* | Control kernel config | Legacy |
| ObserverHandler | OBSERVER_* | State observer config | Legacy |

### CommandContext

Shared utilities passed to all handlers:

```cpp
struct CommandContext {
    uint32_t seq;           // For ACK matching
    CmdType cmdType;        // Current command type
    bool wantAck;           // Whether to send ACK
    EventBus& bus;          // For publishing responses
    ModeManager& mode;      // For state guards

    // Response helpers
    void sendAck(const char* cmd, bool ok, JsonDocument& resp);
    void sendError(const char* cmd, const char* error);

    // State guards
    bool requireIdle(const char* cmdName);
    bool requireArmed(const char* cmdName);

    // ACK cache for duplicate detection
    bool tryReplayAck(CmdType cmd, uint32_t seq);
};
```

### Handler Registration

**Legacy handlers** are registered in `ServiceStorage.initCommands()`:

```cpp
void initCommands() {
    commands = new CommandRegistry(bus, mode, motion);

    safetyHandler = new SafetyHandler(mode);
    motionHandler = new MotionHandler(motion);
    // ... create other handlers

    commands->registerHandler(safetyHandler);
    commands->registerHandler(motionHandler);
    // ... register other handlers

    // Finalize self-registered string handlers
    HandlerRegistry::instance().finalize();
}
```

**New handlers** self-register via static constructors before `main()` runs.

### Typed Command Pattern

Commands use a decode-then-execute pattern for unified JSON/binary handling:

```
┌─────────────────────────────────────────────────────────────────────┐
│                    Unified Command Flow                              │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│   JSON Input                         Binary Input                    │
│   {"vx": 0.5, "omega": 0.1}         [0x10][vx_f32][omega_f32]       │
│        │                                   │                         │
│        ▼                                   ▼                         │
│   decodeSetVelocity(json)           BinaryCommands::decode()        │
│        │                                   │                         │
│        └───────────────┬───────────────────┘                        │
│                        ▼                                             │
│              SetVelocityCmd { vx=0.5, omega=0.1 }                   │
│                        │                                             │
│                        ▼                                             │
│              executeSetVelocity(cmd, ctx)                           │
│                        │                                             │
│                        ▼                                             │
│              motion.setVelocity(...)                                │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

**Typed command structs** (`TypedCommands.h`):
```cpp
namespace mcu::cmd {
    struct SetVelocityCmd {
        float vx = 0.0f;
        float omega = 0.0f;
        bool isValid() const;
    };
}
```

**Decoders** (`CommandDecoders.h`):
```cpp
DecodeResult<SetVelocityCmd> decodeSetVelocity(JsonVariantConst payload);
```

**Handler using pattern**:
```cpp
void handle(CmdType cmd, JsonVariantConst payload, CommandContext& ctx) {
    // 1. Decode
    auto result = decodeSetVelocity(payload);
    if (!result.valid) {
        ctx.sendError("CMD_SET_VEL", result.error);
        return;
    }
    // 2. Execute
    executeSetVelocity(result.cmd, ctx);
}
```

**Benefits:**
- Validation logic in one place (decoder)
- Same executor for JSON and binary paths
- Easier property-based/fuzz testing
- Clear separation of concerns

## FreeRTOS Control Task

### Configuration

```cpp
// In main.cpp
static constexpr bool USE_FREERTOS_CONTROL = true;

// Task configuration
mcu::ControlTaskConfig taskCfg;
taskCfg.rate_hz = 100;      // 10-1000 Hz supported
taskCfg.stack_size = 4096;  // 4KB stack
taskCfg.priority = 5;       // High priority
taskCfg.core = 1;           // Core 1 (Core 0 = WiFi)
```

### What Runs Where

| Component | Location | Rate | Priority |
|-----------|----------|------|----------|
| Motor control, PID | FreeRTOS task | 100Hz | High |
| MotionController | FreeRTOS task | 100Hz | High |
| State estimation | FreeRTOS task | 100Hz | High |
| Safety watchdogs | Main loop | 100Hz | Normal |
| Telemetry | Main loop | 10-50Hz | Normal |
| Command parsing | Main loop | Best effort | Low |
| WiFi/BLE | ESP-IDF tasks | Internal | System |

### Timing Instrumentation

Available via telemetry:

```json
{
  "timing": {
    "safety_us": 45,
    "control_us": 120,
    "total_us": 890,
    "overruns": 0,
    "freertos_ctrl": true,
    "ctrl_task_exec_us": 120,
    "ctrl_task_peak_us": 450,
    "ctrl_task_overruns": 0
  }
}
```

## Dependency Injection

### ServiceContext

```cpp
namespace mcu {
struct ServiceContext {
    // Tier 1: No dependencies
    IClock*         clock = nullptr;    // Time abstraction
    IntentBuffer*   intents = nullptr;  // Command/actuation boundary
    EventBus*       bus = nullptr;
    ModeManager*    mode = nullptr;

    // Tier 2: Motor control
    DcMotorManager*   dcMotor = nullptr;
    MotionController* motion = nullptr;

    // Tier 3: Sensors
    EncoderManager*   encoder = nullptr;
    ImuManager*       imu = nullptr;

    // Tier 4: Communication
    MultiTransport*   transport = nullptr;
    CommandRegistry*  commands = nullptr;

    // Tier 5: Orchestration
    ControlModule*    control = nullptr;
    MCUHost*          host = nullptr;

    // Registries (singletons exposed for DI access)
    HandlerRegistry* handlerRegistry = nullptr;
    ModuleManager*   moduleManager = nullptr;
};
}
```

### Registry Access Pattern

Singletons are still used for static self-registration, but code should access them through ServiceContext:

```cpp
// GOOD: Access via DI
void MyHandler::init(ServiceContext& ctx) {
    ctx.handlerRegistry->setAvailableCaps(caps);
}

// AVOID: Direct singleton access (except in registration)
HandlerRegistry::instance().registerHandler(this);  // Only in REGISTER_HANDLER macro
```

### Setup Module Pattern

```cpp
class ISetupModule {
public:
    virtual const char* name() const = 0;
    virtual bool isCritical() const { return false; }
    virtual Result<void> setup(ServiceContext& ctx) = 0;
};

// Usage in main.cpp - uses central manifest
mcu::ISetupModule** manifest = getSetupManifest();
for (size_t i = 0; i < getSetupManifestSize(); ++i) {
    auto result = manifest[i]->setup(g_ctx);
    if (result.isError() && manifest[i]->isCritical()) {
        // System halts
    }
}
```

#### Central Manifest

All setup modules are defined in `src/setup/SetupManifest.cpp`:

```cpp
// The definitive ordered list of setup modules
mcu::ISetupModule* g_setupManifest[] = {
    nullptr,  // [0] WiFi
    nullptr,  // [1] OTA
    nullptr,  // [2] Safety (CRITICAL)
    nullptr,  // [3] Motors
    nullptr,  // [4] Sensors
    nullptr,  // [5] Transport (CRITICAL)
    nullptr,  // [6] Telemetry
    nullptr   // Null terminator
};
```

**To add a new setup module:**
1. Create `src/setup/SetupXxx.cpp`
2. Add `getSetupXxxModule()` to `include/setup/SetupModules.h`
3. Add one line to the manifest in `SetupManifest.cpp`

### Module Extensibility (Self-Registration)

Similar to the handler extensibility system, modules can now self-register using the `REGISTER_MODULE` macro, reducing the number of files to edit when adding a new module.

**Before (Manual):**
1. Create module class
2. Add to ServiceStorage
3. Add to ServiceContext
4. Manually call `host->addModule()`

**After (Self-Registration):**
1. Create module file with `REGISTER_MODULE` macro - done!

#### IModule Interface

```cpp
class IModule {
public:
    virtual ~IModule() = default;

    // Two-phase initialization for self-registered modules
    virtual void init(mcu::ServiceContext& ctx) {}  // Get dependencies
    virtual void setup() {}                          // One-time setup
    virtual void loop(uint32_t now_ms) {}           // Called every tick
    virtual const char* name() const = 0;
    virtual void handleEvent(const Event& evt) {}
    virtual int priority() const { return 100; }    // Lower = earlier
};
```

#### ModuleManager

The `ModuleManager` singleton collects self-registered modules:

```cpp
class ModuleManager {
public:
    static ModuleManager& instance();

    void registerModule(IModule* module);  // Called by macro
    void finalize();                        // Sort by priority
    void initAll(ServiceContext& ctx);     // Pass dependencies
    void setupAll();                        // Call setup()
    void loopAll(uint32_t now_ms);         // Call loop()
};
```

#### Example: Self-Registered Module

```cpp
// include/module/StatusLedModule.h
#pragma once
#include "core/IModule.h"
#include "core/ModuleMacros.h"
#include "core/ServiceContext.h"

class StatusLedModule : public IModule {
public:
    StatusLedModule() = default;  // Default constructor required

    void init(mcu::ServiceContext& ctx) override {
        bus_ = ctx.bus;  // Get dependencies from context
    }

    void setup() override {
        pinMode(LED_BUILTIN, OUTPUT);
    }

    void loop(uint32_t now_ms) override {
        if (now_ms - lastBlink_ > 500) {
            lastBlink_ = now_ms;
            ledState_ = !ledState_;
            digitalWrite(LED_BUILTIN, ledState_);
        }
    }

    const char* name() const override { return "StatusLedModule"; }
    int priority() const override { return 150; }  // Run after core modules

private:
    EventBus* bus_ = nullptr;
    uint32_t lastBlink_ = 0;
    bool ledState_ = false;
};

REGISTER_MODULE(StatusLedModule);  // Self-registration!
```

#### Dual Module System

MCUHost handles both:
- **Self-registered modules** via `ModuleManager` (new extensible pattern)
- **Manual modules** via `addModule()` (legacy pattern for modules with constructor dependencies)

```cpp
void MCUHost::setup(ServiceContext* ctx) {
    // 1. Initialize self-registered modules
    ModuleManager::instance().finalize();
    if (ctx) ModuleManager::instance().initAll(*ctx);
    ModuleManager::instance().setupAll();

    // 2. Setup manual modules (legacy)
    for (auto* m : modules_) m->setup();
}

void MCUHost::loop(uint32_t now_ms) {
    ModuleManager::instance().loopAll(now_ms);  // Self-registered
    for (auto* m : modules_) m->loop(now_ms);   // Manual
}
```

#### Module Self-Registration Pattern

Modules can self-register their commands, telemetry providers, and event subscriptions in their `init()` method. This eliminates the need to edit multiple files when adding a module.

```cpp
class MyModule : public IModule {
public:
    void init(mcu::ServiceContext& ctx) override {
        // Store dependencies
        bus_ = ctx.bus;
        telemetry_ = ctx.telemetry;

        // 1. Subscribe to events
        // (Note: EventBus requires static trampolines, see EventBus Callback Pattern)

        // 2. Register commands (new string-based API)
        if (ctx.commands) {
            ctx.commands->registerStringHandler(this);  // If module is also a handler
        }

        // 3. Register telemetry providers
        if (ctx.telemetry) {
            ctx.telemetry->registerProvider("mymodule", [this](JsonObject& obj) {
                obj["status"] = status_;
                obj["value"] = currentValue_;
            });
        }
    }

    // ... rest of module implementation
};
```

**Benefits:**
- Adding a module = adding one file
- No edits to ServiceStorage, ServiceContext, or setup files
- Module owns its integration points

### EventBus Callback Pattern

The EventBus uses raw function pointers (not `std::function`) to minimize heap allocation and vtable overhead on embedded. This requires a singleton pattern for event subscribers:

```cpp
// In header
class CommandRegistry {
    static CommandRegistry* s_instance;
    static void handleEventStatic(const Event& evt);  // Raw function pointer
    void handleEvent(const Event& evt);                // Instance method
};

// In cpp
CommandRegistry::CommandRegistry(...) {
    s_instance = this;  // Set singleton in constructor
}

void CommandRegistry::handleEventStatic(const Event& evt) {
    if (s_instance) {
        s_instance->handleEvent(evt);  // Forward to instance
    }
}

// Registration
bus.subscribe(&CommandRegistry::handleEventStatic);
```

**Why not lambdas/std::function?**
- `std::function` allocates heap memory for captures
- Lambdas with captures cannot convert to raw function pointers
- On ESP32, avoiding heap in hot paths improves determinism

**Affected classes:**
- `CommandRegistry`
- `MessageRouter`
- `MCUHost`
- `IdentityModule`
- `LoggingModule`

**Important:** These singletons are ONLY for EventBus callback routing. Always access services through `ServiceContext`, never through `s_instance` directly.

**Future improvement:** Add a `void* context` field to `Event` struct, allowing subscribers to receive context without singletons.

## Error Handling

### Result<T> Pattern

```cpp
namespace mcu {

template<typename T>
class Result {
public:
    static Result ok(T value);
    static Result err(ErrorCode code);

    bool isOk() const;
    bool isError() const;
    T value() const;
    ErrorCode errorCode() const;
};

// TRY macro for propagation
#define TRY(expr) do { \
    auto _r = (expr); \
    if (_r.isError()) return _r.error(); \
} while(0)

}
```

### Error Code Ranges

| Range | Category | Examples |
|-------|----------|----------|
| 0x0001-0x00FF | Generic | InvalidArgument, NotInitialized, Timeout |
| 0x0100-0x01FF | Hardware | HwGpioInvalid, HwI2cTimeout |
| 0x0200-0x02FF | Motor | MotorNotAttached |
| 0x0300-0x03FF | Sensor | SensorNotOnline |
| 0x0500-0x05FF | Safety | SafetyNotArmed, SafetyEstopped |

## Memory Allocation Policy

### Allocation Zones

| Zone | Heap Allowed | Rationale |
|------|--------------|-----------|
| **Setup/Config** | ✅ Yes | One-time initialization, not timing-critical |
| **Command Handlers** | ✅ Yes | JSON parsing inherently allocates, acceptable latency |
| **Control Loop** | ❌ **No** | Must be deterministic, no fragmentation risk |
| **FreeRTOS Task** | ❌ **No** | Real-time control, jitter-sensitive |
| **ISR/Callbacks** | ❌ **No** | Must be bounded-time |
| **Telemetry Emit** | ⚠️ Limited | Pre-allocated buffers only |

### Forbidden in Control Loops

The following are **not allowed** in `runControlLoop()`, `ControlKernel::step()`, or `Observer::update()`:

```cpp
// FORBIDDEN in control loop
new / delete
std::vector::push_back()  // may reallocate
std::string operations    // heap-backed
JsonDocument              // heap-backed
std::make_unique/shared
```

### Pre-allocation Strategy

| Component | Strategy | Location |
|-----------|----------|----------|
| Telemetry buffers | `reserve(256)` | Constructor |
| Transport RX buffers | `reserve(256)` | Constructor |
| SignalBus signals | Fixed-capacity vector | Config-time only |
| Event queue | Fixed-size ring buffer (32) | Compile-time |
| Controller matrices | Fixed arrays `[MAX_STATES]` | Stack/static |

### Implementation Examples

```cpp
// GOOD: Pre-allocated in constructor
TelemetryModule::TelemetryModule(EventBus& bus) : bus_(bus) {
    txBuf_.reserve(256);      // One-time allocation
    sectionBuf_.reserve(128);
}

// GOOD: Reuse member document
void TelemetryModule::sendTelemetry() {
    jsonDoc_.clear();  // Reuse, no allocation
    jsonDoc_["src"] = "mcu";
    // ...
}

// GOOD: Fixed-size arrays in control
class LuenbergerObserver {
    float x_hat_[MAX_STATES];           // Stack array
    float A_[MAX_STATES * MAX_STATES];  // No heap
};

// BAD: Allocation in hot path
void badControlLoop() {
    std::vector<float> temp;  // Heap allocation!
    temp.push_back(value);    // May reallocate!
}
```

### Message Size Limits

| Message Type | Max Size | Enforced |
|--------------|----------|----------|
| JSON command | 512 bytes | Parse buffer |
| Binary command | 64 bytes | Protocol limit |
| Telemetry JSON | 1024 bytes | Serialization buffer |
| Telemetry binary | 256 bytes | Pre-allocated |
| Event payload | 64 bytes | Fixed struct |

### Debugging Allocation Issues

If experiencing random latency spikes or crashes:

1. Check `heap_caps_get_free_size()` before/after suspect operations
2. Enable `CONFIG_HEAP_TRACING` in ESP-IDF for allocation tracking
3. Review telemetry timing stats for periodic spikes
4. Ensure `jsonEnabled_ = false` in production (JSON telemetry allocates)

## Concurrency Model

### Task Architecture

| Task | Core | Priority | Frequency | Purpose |
|------|------|----------|-----------|---------|
| FreeRTOS Control | 1 | 5 | 100Hz | Motor PID, signal updates |
| Main Loop | 0/1 | 1 | Best-effort | Commands, telemetry, comms |

### Shared State Protection

**SignalBus** is accessed from both tasks:
- **Control task**: writes signal values via `set()`, `setWithRateLimit()`
- **Main loop**: reads signals for telemetry via `get()`, `snapshot()`

Thread safety is implemented using ESP32 spinlocks (`portMUX_TYPE`):

```cpp
// Thread-safe single-signal access
bool SignalBus::set(uint16_t id, float v, uint32_t now_ms) {
    enterCritical();  // portENTER_CRITICAL(&lock_)
    // ... modify signal
    exitCritical();   // portEXIT_CRITICAL(&lock_)
    return true;
}

// Thread-safe bulk read for telemetry
size_t snapshot(SignalSnapshot* out, size_t max_count) const;
```

### Usage Guidelines

| Method | Thread Safety | Use Case |
|--------|--------------|----------|
| `get()`, `set()` | Safe | Single-signal access from any task |
| `setWithRateLimit()` | Safe | Rate-limited writes from control task |
| `snapshot()` | Safe | Bulk read for telemetry |
| `all()` | **NOT SAFE** | Setup-time only (returns raw reference) |
| `define()`, `clear()` | **NOT SAFE** | Setup-time only (single-threaded) |

**ModeManager** state transitions are also protected with `portENTER_CRITICAL()`.

## Control System

### Signal Flow

```
Host                          MCU
  │                            │
  │  SET_SIGNAL(vx_ref, 0.5)   │
  ├───────────────────────────►│
  │                            │
  │                     ┌──────┴──────┐
  │                     │  SignalBus  │
  │                     │  vx_ref=0.5 │
  │                     └──────┬──────┘
  │                            │
  │                     ┌──────┴──────┐
  │                     │ ControlKernel│
  │                     │  PID slot 0  │
  │                     │  reads ref,  │
  │                     │  reads meas, │
  │                     │  computes u  │
  │                     └──────┬──────┘
  │                            │
  │                     ┌──────┴──────┐
  │                     │MotionCtrl/  │
  │                     │ DcMotorMgr  │
  │                     │  applies PWM│
  │                     └─────────────┘
```

### Controller Types

| Type | Use Case | Configuration |
|------|----------|---------------|
| PID | Single-input velocity/position | kp, ki, kd, limits |
| StateSpace | Multi-state LQR/observer | K, Kr, Ki, L matrices |

## Loop Rates

### Configurable via Commands

```cpp
struct LoopRates {
    uint16_t ctrl_hz   = 50;   // 5-200 Hz
    uint16_t safety_hz = 100;  // 20-500 Hz
    uint16_t telem_hz  = 10;   // 1-50 Hz
};

// Commands
CMD_CTRL_SET_RATE   {"hz": 100}
CMD_SAFETY_SET_RATE {"hz": 200}
CMD_TELEM_SET_RATE  {"hz": 50}
```

## Testing

### Native Tests (72 test cases)

```bash
pio test -e native
```

| Test Suite | Coverage |
|------------|----------|
| test_protocol | CRC16, frame encode/decode, Python interop |
| test_event_bus | Pub/sub, queue, filtering |
| test_safety | Mode transitions, timeouts |
| test_command_handler_ack | ACK/NAK, sequencing |
| test_result | Result<T> error handling |
| test_golden_path | IntentBuffer command→actuation pipeline |

### Golden Path Integration Test

Verifies the intent-based command pipeline (8 test cases):

```cpp
// test/test_golden_path/test_golden_path.cpp
void test_velocity_intent_set_and_consume();  // Set → consume → empty
void test_velocity_intent_latest_wins();       // Multiple sets → latest consumed
void test_servo_intent_set_and_consume();      // Per-ID servo intents
void test_dc_motor_intent_set_and_consume();   // Per-ID DC motor intents
void test_stepper_intent_set_and_consume();    // Per-ID stepper intents
void test_signal_intent_queue();               // Ring buffer FIFO
void test_clear_all_intents();                 // E-stop clears all
void test_independent_motor_ids();             // IDs don't interfere
```

### Hardware-in-Loop

```bash
# From Python host
pytest tests/test_hil_smoke.py
pytest tests/test_hil_send_commands.py
```

## Memory Usage

```
RAM:   [==        ]  17.0% (55.6 KB / 327.7 KB)
Flash: [=====     ]  46.0% (873.8 KB / 1.9 MB)
```

## Header Organization

### Public API Header

```cpp
// include/mcu.h - Single entry point for external code
#include "mcu.h"  // Includes all public types and forward declarations
```

### Forward Declarations

```cpp
// include/core/Fwd.h - Reduces include chains
namespace mcu {
    class IClock;
    class IntentBuffer;
    struct VelocityIntent;
    // ...
}
```

## Files Structure

```
ESP32 MCU Host/
├── include/
│   ├── mcu.h                  # Public API aggregation header
│   ├── core/
│   │   ├── Fwd.h              # Forward declarations
│   │   ├── Result.h           # Error handling
│   │   ├── ServiceContext.h   # DI container
│   │   ├── ServiceStorage.h   # Instance ownership (header only)
│   │   ├── IntentBuffer.h     # Command/actuation boundary
│   │   ├── RealTimeContract.h # RT_SAFE annotations
│   │   ├── IModule.h          # Module interface with init()
│   │   ├── ModuleManager.h    # Module registry
│   │   ├── ModuleMacros.h     # REGISTER_MODULE macro
│   │   ├── LoopRates.h        # Rate configuration
│   │   └── LoopTiming.h       # Timing instrumentation
│   ├── config/
│   │   ├── RobotConfig.h      # Unified configuration struct
│   │   └── FeatureFlags.h     # Compile-time feature toggles
│   ├── setup/
│   │   ├── ISetupModule.h     # Setup interface
│   │   └── SetupControlTask.h # FreeRTOS task API
│   ├── command/
│   │   ├── CommandRegistry.h  # Command dispatcher (legacy)
│   │   ├── HandlerRegistry.h  # String handler registry (new)
│   │   ├── ICommandHandler.h  # Legacy handler interface
│   │   ├── IStringHandler.h   # New extensible handler interface
│   │   ├── HandlerMacros.h    # REGISTER_HANDLER macro
│   │   ├── CommandContext.h   # Handler context
│   │   ├── ModeManager.h      # Safety state machine
│   │   ├── BinaryCommands.h   # Binary protocol
│   │   └── handlers/          # Domain handlers
│   │       ├── AllHandlers.h  # Include all handlers
│   │       ├── SafetyHandler.h
│   │       ├── MotionHandler.h
│   │       ├── ControlHandler.h
│   │       └── ...
│   ├── sensor/
│   │   ├── ISensor.h          # Sensor interface
│   │   ├── SensorRegistry.h   # Sensor registry
│   │   ├── SensorMacros.h     # REGISTER_SENSOR macro
│   │   ├── AllSensors.h       # Include all sensors
│   │   └── UltrasonicSensor.h # Self-registering ultrasonic
│   ├── transport/
│   │   ├── TransportRegistry.h # Transport registry
│   │   └── ... (transport implementations)
│   ├── motor/
│   │   ├── IActuator.h        # Actuator interface
│   │   ├── ActuatorRegistry.h # Actuator registry
│   │   ├── AllActuators.h     # Include all actuators
│   │   ├── DcMotorActuator.h  # Self-registering DC motor
│   │   └── ... (legacy managers)
│   └── control/
│       ├── SignalBus.h        # Signal routing
│       └── ControlKernel.h    # PID/LQR controllers
├── src/
│   ├── main.cpp               # Entry point (~150 lines)
│   ├── core/
│   │   ├── ServiceStorage.cpp # Heavy init logic (split from .h)
│   │   ├── IntentBuffer.cpp   # Intent buffer implementation
│   │   └── ModuleManager.cpp  # Module registry impl
│   ├── command/
│   │   ├── CommandRegistry.cpp
│   │   └── HandlerRegistry.cpp # Handler registry impl
│   ├── sensor/
│   │   └── SensorRegistry.cpp # Sensor registry impl
│   ├── transport/
│   │   └── TransportRegistry.cpp # Transport registry impl
│   ├── motor/
│   │   └── ActuatorRegistry.cpp # Actuator registry impl
│   ├── setup/                 # Modular setup
│   │   ├── SetupManifest.cpp  # Central module manifest
│   │   ├── SetupWifi.cpp
│   │   ├── SetupSafety.cpp    # Critical
│   │   ├── SetupTransport.cpp # Critical
│   │   ├── SetupMotors.cpp    # Auto-inits actuators
│   │   ├── SetupSensors.cpp   # Auto-inits sensors
│   │   ├── SetupTelemetry.cpp
│   │   └── SetupControlTask.cpp
│   └── loop/
│       ├── LoopControl.cpp
│       └── LoopSafety.cpp
└── test/
    ├── test_protocol/
    ├── test_event_bus/
    ├── test_safety/
    ├── test_result/
    └── test_golden_path/      # IntentBuffer integration tests
```

## Version History

| Version | Changes |
|---------|---------|
| 2.8 | Unified extensibility: SensorRegistry, TransportRegistry, ActuatorRegistry with self-registration (REGISTER_SENSOR, REGISTER_TRANSPORT, REGISTER_ACTUATOR). Deferred init pattern. All components now 1+1 files to add. Grade upgraded to A+. |
| 2.7 | RT_SAFE annotations, IntentBuffer boundary, HandlerCap gating, RobotConfig, header hygiene (mcu.h, Fwd.h), ServiceStorage.cpp split, golden path tests (72 total), registries via ServiceContext |
| 2.6 | Central manifest (SetupManifest.cpp), registerStringHandler() API for module self-registration |
| 2.5 | Module extensibility: IModule.init(), ModuleManager, REGISTER_MODULE macro |
| 2.4 | Handler extensibility: IStringHandler, HandlerRegistry, REGISTER_HANDLER macro |
| 2.3 | SignalBus thread safety with spinlock, snapshot() API |
| 2.2 | Clock interface, memory policy, typed command pattern |
| 2.1 | Command handler refactor: plugin-based domain handlers |
| 2.0 | FreeRTOS control task, timing instrumentation |
| 1.5 | Critical sections, emergency stop callbacks |
| 1.4 | CRC16-CCITT protocol upgrade |
| 1.3 | Result<T> error handling |
| 1.2 | ServiceContext dependency injection |
| 1.1 | ISetupModule pattern |
| 1.0 | Initial architecture |
