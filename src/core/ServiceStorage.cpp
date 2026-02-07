// src/core/ServiceStorage.cpp
// ServiceStorage implementation - service ownership and initialization

#include "core/ServiceStorage.h"
#include "command/HandlerRegistry.h"
#include "core/ModuleManager.h"
#include "sensor/SensorRegistry.h"
#include "transport/TransportRegistry.h"
#include "motor/ActuatorRegistry.h"
#include "config/FeatureFlags.h"

namespace mcu {

ServiceStorage::~ServiceStorage() {
    // Delete in reverse dependency order (Tier 5 → Tier 1)

    // Handlers
    delete observerHandler;
    delete controlHandler;
    delete telemetryHandler;
    delete sensorHandler;
    delete dcMotorHandler;
    delete stepperHandler;
    delete servoHandler;
    delete gpioHandler;
    delete motionHandler;
    delete safetyHandler;

    // Modules
    delete identity;

    // Tier 5
    delete host;
    delete control;

    // Tier 4
    delete commands;
    delete router;
    delete ble;
    delete wifi;
    delete uart;
}

void ServiceStorage::initTransports(HardwareSerial& serial, uint32_t baud, uint16_t tcpPort) {
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

void ServiceStorage::initRouter() {
    router = new MessageRouter(bus, transport);
}

void ServiceStorage::initCommands() {
    commands = new CommandRegistry(bus, mode, motion);
    commands->setIntentBuffer(&intents);
    commands->setHandlerRegistry(&HandlerRegistry::instance());  // Explicit wiring

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

    // Register legacy handlers
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

    // Set available capabilities from feature flags
    HandlerRegistry::instance().setAvailableCaps(buildCapabilityMask());

    // Finalize self-registered string handlers
    HandlerRegistry::instance().finalize();
}

void ServiceStorage::initControl() {
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

void ServiceStorage::initHost(const char* deviceName) {
    host = new MCUHost(bus, router);
    identity = new IdentityModule(bus, transport, deviceName);
}

ServiceContext ServiceStorage::buildContext() {
    ServiceContext ctx;

    // Tier 1
    ctx.clock   = &clock;
    ctx.intents = &intents;
    ctx.bus     = &bus;
    ctx.mode    = &mode;
    ctx.gpio    = &gpio;
    ctx.pwm     = &pwm;

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

    // Registries (singletons exposed via context for DI)
    ctx.handlerRegistry   = &HandlerRegistry::instance();
    ctx.moduleManager     = &ModuleManager::instance();
    ctx.sensorRegistry    = &SensorRegistry::instance();
    ctx.transportRegistry = &TransportRegistry::instance();
    ctx.actuatorRegistry  = &ActuatorRegistry::instance();

    return ctx;
}

} // namespace mcu
