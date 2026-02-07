# MARA Firmware

```
 ███╗   ███╗ █████╗ ██████╗  █████╗
 ████╗ ████║██╔══██╗██╔══██╗██╔══██╗
 ██╔████╔██║███████║██████╔╝███████║
 ██║╚██╔╝██║██╔══██║██╔══██╗██╔══██║
 ██║ ╚═╝ ██║██║  ██║██║  ██║██║  ██║
 ╚═╝     ╚═╝╚═╝  ╚═╝╚═╝  ╚═╝╚═╝  ╚═╝
 Modular Asynchronous Robotics Architecture
```

**ESP32 Firmware Component**

[![PlatformIO](https://img.shields.io/badge/PlatformIO-ESP32-orange)](https://platformio.org/)
[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

---

## Overview

**MARA** (Modular Asynchronous Robotics Architecture) is a complete robotics control framework consisting of:

| Component | Repository | Description |
|-----------|------------|-------------|
| **Firmware** | `ESP32 MCU Host` (this repo) | Real-time motor control, sensor fusion, communication |
| **Host** | [`robot_host`](../../../Host) | Python async client, telemetry, research tools |

This repository contains the **ESP32 firmware** - a modular, configurable firmware for ESP32-based robot control systems. Supports differential drive robots, sensors (IMU, encoders, LIDAR, ultrasonic), and multiple communication transports (USB Serial, WiFi, Bluetooth, MQTT).

## Key Features

- **Modular Architecture**: Enable/disable features via compile-time flags
- **Self-Registration**: Add handlers, modules, sensors, transports, and actuators with a single macro
- **Multiple Transports**: USB Serial, WiFi TCP, Bluetooth Classic, MQTT
- **Motor Control**: DC motors, servos, steppers with motion controller
- **Sensors**: IMU (MPU6050), encoders, ultrasonic, LIDAR (VL53L0X)
- **Control System**: Signal bus, PID controllers, state observers
- **Telemetry**: Binary and JSON telemetry streaming at 50+ Hz
- **Safety**: E-STOP, watchdog, connection monitoring

## Quick Start

### Prerequisites

- [PlatformIO](https://platformio.org/) (VS Code extension or CLI)
- ESP32 development board (ESP32-DevKitC, ESP32-S3, etc.)
- USB cable for programming

### Build & Upload

```bash
# Build the full firmware
pio run -e esp32_usb

# Upload to ESP32
pio run -e esp32_usb -t upload

# Monitor serial output
pio device monitor -b 115200
```

### Build Profiles

| Profile | Description | Flash Size |
|---------|-------------|------------|
| `esp32_minimal` | UART only, bare minimum | ~350KB |
| `esp32_motors` | Motors + encoders, no network | ~550KB |
| `esp32_sensors` | WiFi + all sensors, no motors | ~600KB |
| `esp32_control` | Full control system | ~750KB |
| `esp32_full` | Everything enabled | ~870KB |
| `esp32_usb` | Full + USB upload (default) | ~870KB |
| `esp32_ota` | Full + OTA upload | ~870KB |

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                         main.cpp                                 │
├─────────────┬─────────────┬─────────────┬─────────────┬─────────┤
│  Transport  │   Command   │   Control   │   Module    │   HW    │
│  Layer      │   Layer     │   Layer     │   Layer     │  Layer  │
├─────────────┼─────────────┼─────────────┼─────────────┼─────────┤
│ UartTrans   │ CmdHandler  │ SignalBus   │ Telemetry   │ GPIO    │
│ WifiTrans   │ MsgRouter   │ CtrlKernel  │ Heartbeat   │ PWM     │
│ BleTrans    │ ModeManager │ Observer    │ Identity    │ Safety  │
│ MqttTrans   │             │ PID         │ Logging     │         │
├─────────────┴─────────────┴─────────────┴─────────────┴─────────┤
│                         EventBus                                 │
├─────────────┬─────────────┬─────────────┬─────────────┬─────────┤
│ DcMotor     │ Servo       │ IMU         │ Encoder     │ Lidar   │
│ Manager     │ Manager     │ Manager     │ Manager     │ Manager │
└─────────────┴─────────────┴─────────────┴─────────────┴─────────┘
```

## Directory Structure

```
ESP32 MCU Host/
├── include/
│   ├── config/          # Build configuration
│   │   ├── FeatureFlags.h   # Feature flag definitions
│   │   ├── PinConfig.h      # Pin assignments
│   │   └── LoopRates.h      # Timing configuration
│   ├── core/            # Core interfaces
│   │   ├── Event.h          # Event system
│   │   ├── EventBus.h       # Pub/sub messaging
│   │   ├── IModule.h        # Module interface
│   │   ├── ModuleManager.h  # Module registry (extensible)
│   │   ├── ModuleMacros.h   # REGISTER_MODULE macro
│   │   └── Protocol.h       # Frame protocol
│   ├── transport/       # Communication transports
│   │   ├── IRegisteredTransport.h  # Self-registration interface
│   │   ├── TransportRegistry.h     # Transport registry
│   │   ├── UartTransport.h
│   │   ├── WifiTransport.h
│   │   ├── BleTransport.h
│   │   └── MqttTransport.h
│   ├── motor/           # Motor drivers + actuators
│   │   ├── IActuator.h         # Self-registration interface
│   │   ├── ActuatorRegistry.h  # Actuator registry
│   │   ├── DcMotorActuator.h   # Self-registering DC motor
│   │   ├── DcMotorManager.h
│   │   ├── ServoManager.h
│   │   ├── StepperManager.h
│   │   └── MotionController.h
│   ├── sensor/          # Sensor interfaces
│   │   ├── ISensor.h           # Self-registration interface
│   │   ├── SensorRegistry.h    # Sensor registry
│   │   ├── ImuManager.h
│   │   ├── EncoderManager.h
│   │   ├── UltrasonicManager.h
│   │   └── LidarManager.h
│   ├── control/         # Control system
│   │   ├── SignalBus.h      # Signal routing
│   │   ├── ControlKernel.h  # Control loop
│   │   └── Observer.h       # State estimation
│   ├── command/         # Command handling
│   │   ├── CommandRegistry.h   # Legacy dispatcher
│   │   ├── HandlerRegistry.h   # New extensible registry
│   │   ├── IStringHandler.h    # New handler interface
│   │   ├── HandlerMacros.h     # REGISTER_HANDLER macro
│   │   ├── MessageRouter.h
│   │   └── ModeManager.h
│   ├── module/          # System modules
│   │   ├── TelemetryModule.h
│   │   ├── HeartbeatModule.h
│   │   └── IdentityModule.h
│   └── hw/              # Hardware abstraction
│       ├── GpioManager.h
│       ├── PwmManager.h
│       └── SafetyManager.h
├── src/                 # Implementation files
│   └── (mirrors include structure)
├── test/                # Unit tests
└── platformio.ini       # Build configuration
```

## Configuration

### Feature Flags (`include/config/FeatureFlags.h`)

Enable/disable features at compile time:

```cpp
// Transport
#define HAS_WIFI             1
#define HAS_BLE              0
#define HAS_MQTT_TRANSPORT   0

// Motors
#define HAS_DC_MOTOR         1
#define HAS_SERVO            1
#define HAS_ENCODER          1
#define HAS_MOTION_CONTROLLER 1

// Sensors
#define HAS_IMU              1
#define HAS_LIDAR            0
#define HAS_ULTRASONIC       1

// Control
#define HAS_SIGNAL_BUS       1
#define HAS_CONTROL_KERNEL   1

// System
#define HAS_TELEMETRY        1
#define HAS_HEARTBEAT        1
```

### Pin Configuration (`include/config/PinConfig.h`)

Define hardware pin assignments:

```cpp
// Motor pins
#define MOTOR_L_IN1   25
#define MOTOR_L_IN2   26
#define MOTOR_L_PWM   27
#define MOTOR_R_IN1   32
#define MOTOR_R_IN2   33
#define MOTOR_R_PWM   14

// Encoder pins
#define ENC_L_A       34
#define ENC_L_B       35
#define ENC_R_A       36
#define ENC_R_B       39

// IMU (I2C)
#define IMU_SDA       21
#define IMU_SCL       22
```

## Extensibility

MARA uses self-registration patterns to minimize boilerplate when adding components:

| Component | Macro | Files to Edit |
|-----------|-------|---------------|
| Command Handler | `REGISTER_HANDLER(ClassName)` | 1 (+ include) |
| Module | `REGISTER_MODULE(ClassName)` | 1 (+ include) |
| Sensor | `REGISTER_SENSOR(ClassName)` | 1 (+ include) |
| Transport | `REGISTER_TRANSPORT(ClassName)` | 1 (+ include) |
| Actuator | `REGISTER_ACTUATOR(ClassName)` | 1 (+ include) |

Example (adding a new command handler):

```cpp
// include/command/handlers/MyHandler.h
#include "command/IStringHandler.h"
#include "command/HandlerMacros.h"

class MyHandler : public IStringHandler {
public:
    static constexpr const char* CMDS[] = {"CMD_MY_COMMAND", nullptr};
    const char* const* commands() const override { return CMDS; }
    const char* name() const override { return "MyHandler"; }

    void handle(const char* cmd, JsonVariantConst payload, CommandContext& ctx) override {
        // Handle command...
        ctx.sendAck(cmd, true, JsonDocument{});
    }
};

REGISTER_HANDLER(MyHandler);  // Auto-registers at startup
```

See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for detailed extensibility documentation.

## Protocol

MARA uses a binary framing protocol for efficient communication:

```
[HEADER][LEN_HI][LEN_LO][MSG_TYPE][PAYLOAD...][CHECKSUM]
  0xAA     1B      1B       1B       N bytes     1B
```

### Message Types

| Type | Value | Description |
|------|-------|-------------|
| HEARTBEAT | 0x01 | Keep-alive |
| PING | 0x02 | Ping request |
| PONG | 0x03 | Ping response |
| VERSION_REQUEST | 0x04 | Request firmware info |
| VERSION_RESPONSE | 0x05 | Firmware info response |
| TELEMETRY_BIN | 0x30 | Binary telemetry |
| CMD_JSON | 0x50 | JSON command |

### Commands

See `include/docs/commandSets.md` for full command reference.

Common commands:
- `CMD_ARM` / `CMD_DISARM` - Enable/disable motor control
- `CMD_ACTIVATE` / `CMD_DEACTIVATE` - Start/stop motors
- `CMD_SET_VEL` - Set velocity (vx, omega)
- `CMD_ESTOP` / `CMD_CLEAR_ESTOP` - Emergency stop
- `CMD_TELEMETRY_ON` / `CMD_TELEMETRY_OFF` - Enable telemetry

## Testing

```bash
# Run all native unit tests
pio test -e native

# Run specific test
pio test -e native -f test_protocol

# Verbose output
pio test -e native -v
```

## MARA Host Integration

This firmware is designed to work with the [MARA Host](../../../Host) Python package:

```python
from robot_host.transport.serial_transport import SerialTransport
from robot_host.command.client import AsyncRobotClient

transport = SerialTransport("/dev/ttyUSB0", baudrate=115200)
client = AsyncRobotClient(transport=transport)

await client.start()
await client.arm()
await client.activate()
await client.set_vel(vx=0.2, omega=0.0)
```

## Performance

| Metric | Value |
|--------|-------|
| Control loop rate | 100 Hz |
| Telemetry rate | 50 Hz |
| Command latency | < 5ms |
| Signal lookup | O(1) |

## License

MIT License
