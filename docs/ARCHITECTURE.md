# MARA Firmware Architecture

**Modular Asynchronous Robotics Architecture - ESP32 Firmware Component**

**Architecture Grade: A-** (Production-ready for non-safety-critical robotics)

---

## Overview

The MARA Firmware follows a layered, real-time architecture with:
- **FreeRTOS control task** for deterministic motor control
- **Dependency injection** via ServiceContext
- **Result<T> error handling** for robust error propagation
- **Modular setup** via ISetupModule pattern
- **CRC16-CCITT** protocol integrity

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
| Hardware watchdog | 2 second timeout | ESP32 task watchdog |
| Host timeout | 2 second default | ModeManager.update() |
| Motion timeout | 2 second default | ModeManager.update() |
| Emergency stop | Direct motor disable | SetupSafety callback |
| Critical module halt | System stops on failure | main.cpp setup |

### Critical Modules

These modules are marked `isCritical() = true`. If they fail during setup, the system halts:

- **SetupSafety** - ModeManager must be operational
- **SetupTransport** - Must be able to receive commands

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

## Command Dispatch Architecture

### Plugin-Based Handler Pattern

Commands are processed through a registry that dispatches to domain-specific handlers:

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
│                   parseJsonToMessage() ──► CmdType enum              │
│                          │                                           │
│                          ▼                                           │
│                   findHandler(cmdType)                               │
│                          │                                           │
│          ┌───────────────┼───────────────┐                          │
│          ▼               ▼               ▼                          │
│   SafetyHandler   MotionHandler   ControlHandler  ...               │
│   (ARM, ESTOP)    (SET_VEL)       (CTRL_SLOT_*)                     │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

### ICommandHandler Interface

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

| Handler | Commands | Responsibility |
|---------|----------|----------------|
| SafetyHandler | ARM, DISARM, ESTOP, CLEAR_ESTOP | Safety state transitions |
| MotionHandler | SET_VEL, STOP | Differential drive control |
| GpioHandler | GPIO_WRITE, GPIO_READ, GPIO_TOGGLE | Digital I/O |
| ServoHandler | SERVO_ATTACH, SERVO_SET | Servo motor control |
| StepperHandler | STEPPER_ENABLE, STEPPER_MOVE | Stepper motor control |
| DcMotorHandler | DC_SET_SPEED, DC_VEL_PID_* | DC motor + velocity PID |
| SensorHandler | ULTRASONIC_*, ENCODER_* | Sensor configuration |
| TelemetryHandler | TELEM_SET_INTERVAL | Telemetry settings |
| ControlHandler | CTRL_SLOT_*, CTRL_SIGNAL_* | Control kernel config |
| ObserverHandler | OBSERVER_* | State observer config |

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

Handlers are registered in `ServiceStorage.initCommands()`:

```cpp
void initCommands() {
    commands = new CommandRegistry(bus, mode, motion);

    safetyHandler = new SafetyHandler(mode);
    motionHandler = new MotionHandler(motion);
    // ... create other handlers

    commands->registerHandler(safetyHandler);
    commands->registerHandler(motionHandler);
    // ... register other handlers
}
```

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
};
}
```

### Setup Module Pattern

```cpp
class ISetupModule {
public:
    virtual const char* name() const = 0;
    virtual bool isCritical() const { return false; }
    virtual Result<void> setup(ServiceContext& ctx) = 0;
};

// Usage in main.cpp
for (auto* mod : setupModules) {
    auto result = mod->setup(g_ctx);
    if (result.isError() && mod->isCritical()) {
        // System halts
    }
}
```

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

### Native Tests (64 test cases)

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

## Files Structure

```
ESP32 MCU Host/
├── include/
│   ├── core/
│   │   ├── Result.h           # Error handling
│   │   ├── ServiceContext.h   # DI container
│   │   ├── ServiceStorage.h   # Instance ownership
│   │   ├── LoopRates.h        # Rate configuration
│   │   └── LoopTiming.h       # Timing instrumentation
│   ├── setup/
│   │   ├── ISetupModule.h     # Setup interface
│   │   └── SetupControlTask.h # FreeRTOS task API
│   ├── command/
│   │   ├── CommandRegistry.h  # Command dispatcher
│   │   ├── CommandContext.h   # Handler context
│   │   ├── ICommandHandler.h  # Handler interface
│   │   ├── ModeManager.h      # Safety state machine
│   │   ├── BinaryCommands.h   # Binary protocol
│   │   └── handlers/          # Domain handlers
│   │       ├── SafetyHandler.h
│   │       ├── MotionHandler.h
│   │       ├── ControlHandler.h
│   │       └── ...
│   └── control/
│       ├── SignalBus.h        # Signal routing
│       └── ControlKernel.h    # PID/LQR controllers
├── src/
│   ├── main.cpp               # Entry point (~150 lines)
│   ├── setup/                 # Modular setup
│   │   ├── SetupWifi.cpp
│   │   ├── SetupSafety.cpp    # Critical
│   │   ├── SetupTransport.cpp # Critical
│   │   ├── SetupMotors.cpp
│   │   ├── SetupSensors.cpp
│   │   ├── SetupTelemetry.cpp
│   │   └── SetupControlTask.cpp
│   └── loop/
│       ├── LoopControl.cpp
│       └── LoopSafety.cpp
└── test/
    ├── test_protocol/
    ├── test_event_bus/
    ├── test_safety/
    └── test_result/
```

## Version History

| Version | Changes |
|---------|---------|
| 2.1 | Command handler refactor: plugin-based domain handlers |
| 2.0 | FreeRTOS control task, timing instrumentation |
| 1.5 | Critical sections, emergency stop callbacks |
| 1.4 | CRC16-CCITT protocol upgrade |
| 1.3 | Result<T> error handling |
| 1.2 | ServiceContext dependency injection |
| 1.1 | ISetupModule pattern |
| 1.0 | Initial architecture |
