#pragma once

#include <Arduino.h>  // For HardwareSerial
#include "core/ITransport.h"

// Include all necessary headers for service types
#include "core/EventBus.h"
#include "core/MCUHost.h"
#include "core/LoopScheduler.h"
#include "core/ServiceContext.h"

#include "command/ModeManager.h"
#include "command/MessageRouter.h"
#include "command/CommandRegistry.h"
#include "command/handlers/AllHandlers.h"

#include "hw/GpioManager.h"
#include "hw/PwmManager.h"

#include "motor/DcMotorManager.h"
#include "motor/ServoManager.h"
#include "motor/StepperManager.h"
#include "motor/MotionController.h"

#include "sensor/EncoderManager.h"
#include "sensor/ImuManager.h"
#include "sensor/LidarManager.h"
#include "sensor/UltrasonicManager.h"

#include "transport/MultiTransport.h"
#include "transport/UartTransport.h"
#include "transport/WifiTransport.h"
#include "transport/BleTransport.h"

#include "module/HearbeatModule.h"
#include "module/LoggingModule.h"
#include "module/IdentityModule.h"
#include "module/TelemetryModule.h"
#include "module/ControlModule.h"

namespace mcu {

/// ServiceStorage owns all service instances.
/// This replaces global variables with a single owning struct.
/// Services are constructed in dependency order.
///
/// Usage:
///   static ServiceStorage storage;
///   storage.init();
///   ServiceContext ctx = storage.buildContext();
struct ServiceStorage {
    // =========================================================================
    // Tier 1: Core infrastructure (no dependencies)
    // =========================================================================
    EventBus    bus;
    ModeManager mode;
    GpioManager gpio;
    PwmManager  pwm;

    // =========================================================================
    // Tier 2: Motor control
    // Constructors take references, so we initialize after gpio/pwm
    // =========================================================================
    DcMotorManager dcMotor{gpio, pwm};
    ServoManager   servo;
    StepperManager stepper{gpio};

    // MotionController needs motors, servo, stepper
    // Default values - can be reconfigured in init()
    MotionController motion{
        dcMotor,
        /*leftMotorId=*/0,
        /*rightMotorId=*/1,
        /*wheelBase=*/0.25f,
        /*maxLinear=*/0.5f,
        /*maxAngular=*/2.0f,
        &servo,
        &stepper
    };

    // =========================================================================
    // Tier 3: Sensors
    // =========================================================================
    EncoderManager    encoder;
    ImuManager        imu;
    LidarManager      lidar;
    UltrasonicManager ultrasonic;

    // =========================================================================
    // Tier 4: Communication
    // =========================================================================
    MultiTransport transport;

    // Note: UartTransport/WifiTransport need HardwareSerial/port at construction
    // These will be initialized separately since they need Arduino runtime
    UartTransport* uart = nullptr;
    WifiTransport* wifi = nullptr;
    BleTransport*  ble  = nullptr;

    // Router depends on bus and transport
    MessageRouter* router = nullptr;

    // Registry depends on bus, mode, motion
    CommandRegistry* commands = nullptr;

    // Telemetry depends on bus
    TelemetryModule telemetry{bus};

    // =========================================================================
    // Tier 5: Orchestration
    // =========================================================================
    // Control module depends on many services
    ControlModule* control = nullptr;

    // Host depends on bus and router
    MCUHost* host = nullptr;

    // =========================================================================
    // Modules
    // =========================================================================
    HeartbeatModule heartbeat{bus};
    LoggingModule   logger{bus};
    IdentityModule* identity = nullptr;

    // =========================================================================
    // Command Handlers
    // =========================================================================
    SafetyHandler*    safetyHandler    = nullptr;
    MotionHandler*    motionHandler    = nullptr;
    GpioHandler*      gpioHandler      = nullptr;
    ServoHandler*     servoHandler     = nullptr;
    StepperHandler*   stepperHandler   = nullptr;
    DcMotorHandler*   dcMotorHandler   = nullptr;
    SensorHandler*    sensorHandler    = nullptr;
    TelemetryHandler* telemetryHandler = nullptr;
    ControlHandler*   controlHandler   = nullptr;
    ObserverHandler*  observerHandler  = nullptr;

    // =========================================================================
    // Loop Schedulers
    // =========================================================================
    LoopScheduler safetyScheduler{20};    // 20ms = 50Hz
    LoopScheduler controlScheduler{20};   // 20ms = 50Hz
    LoopScheduler telemetryScheduler{100}; // 100ms = 10Hz

    // =========================================================================
    // Initialization methods
    // =========================================================================

    /// Initialize transports that require runtime parameters.
    /// Call this in setup() after Serial is ready.
    void initTransports(HardwareSerial& serial, uint32_t baud, uint16_t tcpPort) {
        uart = new UartTransport(serial, baud);
        wifi = new WifiTransport(tcpPort);
#if HAS_BLE
        ble = new BleTransport("ESP32-SPP");
#endif
        if (uart) transport.addTransport(uart);
        if (wifi) transport.addTransport(wifi);
#if HAS_BLE
        if (ble) transport.addTransport(ble);
#endif
    }

    /// Initialize router (call after transports are set up).
    void initRouter() {
        router = new MessageRouter(bus, transport);
    }

    /// Initialize command registry and handlers.
    void initCommands() {
        commands = new CommandRegistry(bus, mode, motion);

        // Create handlers
        safetyHandler    = new SafetyHandler(mode);
        motionHandler    = new MotionHandler(motion);
        gpioHandler      = new GpioHandler(gpio, pwm);
        servoHandler     = new ServoHandler(servo, motion);
        stepperHandler   = new StepperHandler(stepper, motion);
        dcMotorHandler   = new DcMotorHandler(dcMotor);
        sensorHandler    = new SensorHandler(ultrasonic, encoder);
        telemetryHandler = new TelemetryHandler(telemetry);
        controlHandler   = new ControlHandler();
        observerHandler  = new ObserverHandler();

        // Register handlers
        commands->registerHandler(safetyHandler);
        commands->registerHandler(motionHandler);
        commands->registerHandler(gpioHandler);
        commands->registerHandler(servoHandler);
        commands->registerHandler(stepperHandler);
        commands->registerHandler(dcMotorHandler);
        commands->registerHandler(sensorHandler);
        commands->registerHandler(telemetryHandler);
        commands->registerHandler(controlHandler);
        commands->registerHandler(observerHandler);
    }

    /// Initialize control module.
    void initControl() {
        control = new ControlModule(
            &bus,
            &mode,
            &motion,
            &encoder,
            &imu,
            &telemetry
        );

        // Wire control module to handlers
        if (controlHandler) {
            controlHandler->setControlModule(control);
        }
        if (observerHandler) {
            observerHandler->setControlModule(control);
        }
        if (commands) {
            commands->setControlModule(control);
        }
    }

    /// Initialize host and identity module.
    void initHost(const char* deviceName) {
        host = new MCUHost(bus, router);
        identity = new IdentityModule(bus, transport, deviceName);
    }

    /// Build a ServiceContext from this storage.
    /// Call after all init methods have been called.
    ServiceContext buildContext() {
        ServiceContext ctx;

        // Tier 1
        ctx.bus  = &bus;
        ctx.mode = &mode;
        ctx.gpio = &gpio;
        ctx.pwm  = &pwm;

        // Tier 2
        ctx.dcMotor = &dcMotor;
        ctx.servo   = &servo;
        ctx.stepper = &stepper;
        ctx.motion  = &motion;

        // Tier 3
        ctx.encoder    = &encoder;
        ctx.imu        = &imu;
        ctx.lidar      = &lidar;
        ctx.ultrasonic = &ultrasonic;

        // Tier 4
        ctx.transport = &transport;
        ctx.uart      = uart;
        ctx.wifi      = wifi;
        ctx.ble       = ble;
        ctx.router    = router;
        ctx.commands  = commands;
        ctx.telemetry = &telemetry;

        // Tier 5
        ctx.control = control;
        ctx.host    = host;

        // Modules
        ctx.heartbeat = &heartbeat;
        ctx.logger    = &logger;
        ctx.identity  = identity;
        if (control) {
            ctx.observers = &control->observers();
        }

        // Handlers
        ctx.safetyHandler    = safetyHandler;
        ctx.motionHandler    = motionHandler;
        ctx.gpioHandler      = gpioHandler;
        ctx.servoHandler     = servoHandler;
        ctx.stepperHandler   = stepperHandler;
        ctx.dcMotorHandler   = dcMotorHandler;
        ctx.sensorHandler    = sensorHandler;
        ctx.telemetryHandler = telemetryHandler;
        ctx.controlHandler   = controlHandler;
        ctx.observerHandler  = observerHandler;

        // Schedulers
        ctx.safetyScheduler    = &safetyScheduler;
        ctx.controlScheduler   = &controlScheduler;
        ctx.telemetryScheduler = &telemetryScheduler;

        return ctx;
    }
};

} // namespace mcu
