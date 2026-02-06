#pragma once

// Forward declarations for all service types
class EventBus;
class ModeManager;
class GpioManager;
class PwmManager;
class DcMotorManager;
class ServoManager;
class StepperManager;
class MotionController;
class EncoderManager;
class ImuManager;
class LidarManager;
class UltrasonicManager;
class MultiTransport;
class MessageRouter;
class CommandRegistry;
class TelemetryModule;
class ControlModule;
class MCUHost;
class LoopScheduler;
class ObserverManager;

// Transport types
class UartTransport;
class WifiTransport;
class BleTransport;

// Base class types (using base classes avoids needing inheritance info)
class ICommandHandler;
class IModule;

namespace mcu {

/// ServiceContext provides dependency injection for all services.
/// Instead of using globals, services receive a pointer to this context
/// and access their dependencies through it.
///
/// Services are organized by initialization tier:
/// - Tier 1: No dependencies (core infrastructure)
/// - Tier 2: Motor control (depend on Tier 1)
/// - Tier 3: Sensors
/// - Tier 4: Communication
/// - Tier 5: Orchestration (depend on multiple tiers)
struct ServiceContext {
    // =========================================================================
    // Tier 1: Core infrastructure (no dependencies)
    // =========================================================================
    EventBus*       bus  = nullptr;
    ModeManager*    mode = nullptr;
    GpioManager*    gpio = nullptr;
    PwmManager*     pwm  = nullptr;

    // =========================================================================
    // Tier 2: Motor control (depend on GPIO, PWM)
    // =========================================================================
    DcMotorManager*   dcMotor  = nullptr;
    ServoManager*     servo    = nullptr;
    StepperManager*   stepper  = nullptr;
    MotionController* motion   = nullptr;

    // =========================================================================
    // Tier 3: Sensors
    // =========================================================================
    EncoderManager*    encoder    = nullptr;
    ImuManager*        imu        = nullptr;
    LidarManager*      lidar      = nullptr;
    UltrasonicManager* ultrasonic = nullptr;

    // =========================================================================
    // Tier 4: Communication
    // =========================================================================
    MultiTransport*  transport = nullptr;
    MessageRouter*   router    = nullptr;
    CommandRegistry* commands  = nullptr;
    TelemetryModule* telemetry = nullptr;

    // Individual transports (optional, for direct access)
    UartTransport* uart = nullptr;
    WifiTransport* wifi = nullptr;
    BleTransport*  ble  = nullptr;

    // =========================================================================
    // Tier 5: Orchestration
    // =========================================================================
    ControlModule* control = nullptr;
    MCUHost*       host    = nullptr;

    // =========================================================================
    // Modules (using IModule* for polymorphic access)
    // =========================================================================
    IModule* heartbeat = nullptr;
    IModule* logger    = nullptr;
    IModule* identity  = nullptr;
    ObserverManager* observers = nullptr;

    // =========================================================================
    // Command handlers (using ICommandHandler* for polymorphic access)
    // =========================================================================
    ICommandHandler* safetyHandler    = nullptr;
    ICommandHandler* motionHandler    = nullptr;
    ICommandHandler* gpioHandler      = nullptr;
    ICommandHandler* servoHandler     = nullptr;
    ICommandHandler* stepperHandler   = nullptr;
    ICommandHandler* dcMotorHandler   = nullptr;
    ICommandHandler* sensorHandler    = nullptr;
    ICommandHandler* telemetryHandler = nullptr;
    ICommandHandler* controlHandler   = nullptr;
    ICommandHandler* observerHandler  = nullptr;

    // =========================================================================
    // Loop Schedulers
    // =========================================================================
    LoopScheduler* safetyScheduler   = nullptr;
    LoopScheduler* controlScheduler  = nullptr;
    LoopScheduler* telemetryScheduler = nullptr;

    // =========================================================================
    // Convenience methods for null-safety
    // =========================================================================
    bool hasMotorControl() const {
        return dcMotor != nullptr && motion != nullptr;
    }

    bool hasSensors() const {
        return encoder != nullptr || imu != nullptr || lidar != nullptr;
    }

    bool hasTransport() const {
        return transport != nullptr && router != nullptr;
    }

    bool isValid() const {
        return bus != nullptr && mode != nullptr;
    }
};

} // namespace mcu
