#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>

#include "core/EventBus.h"
#include "core/MCUHost.h"
#include "transport/MultiTransport.h"
#include "transport/UartTransport.h"
#include "transport/WifiTransport.h"
#include "command/MessageRouter.h"
#include "core/LoopRates.h"
#include "core/LoopScheduler.h"
#include "module/HearbeatModule.h"
#include "module/LoggingModule.h"
#include "config/WifiSecrets.h"
#include "transport/BleTransport.h"
#include "module/IdentityModule.h"
#include "command/ModeManager.h"
#include "command/CommandHandler.h"
#include "motor/MotionController.h"
#include "config/PinConfig.h"
#include "hw/GpioManager.h"
#include "hw/PwmManager.h"
#include "motor/ServoManager.h"
#include "motor/StepperManager.h"
#include "motor/DcMotorManager.h"
#include "config/GpioChannelDefs.h"
#include "module/TelemetryModule.h"
#include "sensor/UltrasonicManager.h"
#include "sensor/ImuManager.h"
#include "sensor/LidarManager.h"
#include "sensor/EncoderManager.h"
#include "module/ControlModule.h"
#include "control/Observer.h"

// Optional: Control kernel (comment out if not using yet)
// #include "control/SignalBus.h"
// #include "control/ControlKernel.h"
// #include "module/ControlModule.h"

// -----------------------------------------------------------------------------
// WiFi defaults (if WifiSecrets.h not configured)
// -----------------------------------------------------------------------------
#ifndef WIFI_STA_SSID
#define WIFI_STA_SSID "YourHomeSSID"
#endif

#ifndef WIFI_STA_PASSWORD
#define WIFI_STA_PASSWORD "YourHomePassword"
#endif

// AP = robot's own network
const char* AP_SSID  = "RobotAP";
const char* AP_PASS  = "robotpass";

// -----------------------------------------------------------------------------
// Globals
// -----------------------------------------------------------------------------

// Core bus + mode
EventBus    g_bus;
ModeManager g_modeManager;

// Hardware managers
GpioManager    g_gpioManager;
PwmManager     g_pwmManager;
DcMotorManager g_dcMotorManager(g_gpioManager, g_pwmManager);
ServoManager   g_servoManager;
StepperManager g_stepperManager(g_gpioManager);

// Sensor managers
UltrasonicManager g_ultrasonicManager;
ImuManager        g_imu;
LidarManager      g_lidar;
EncoderManager    g_encoder;

// Motion controller (diff drive + servo interpolation + stepper passthrough)
MotionController g_motionController(
    g_dcMotorManager,
    /*leftMotorId=*/0,
    /*rightMotorId=*/1,
    /*wheelBase=*/0.25f,   // meters
    /*maxLinear=*/0.5f,    // m/s
    /*maxAngular=*/2.0f,   // rad/s
    &g_servoManager,
    &g_stepperManager
);

// Transports
MultiTransport g_multiTransport;

// Dedicated UART1 for protocol
HardwareSerial SerialPort(1);          // UART1
UartTransport  g_uart(Serial, 115200); // protocol over USB Serial
WifiTransport  g_wifi(3333);           // TCP port
#if HAS_BLE
BleTransport   g_ble("ESP32-SPP");
#endif

// Router + host
MessageRouter g_router(g_bus, g_multiTransport);
MCUHost       g_host(g_bus, &g_router);

// Telemetry
TelemetryModule g_telemetry(g_bus);

// Command handler
CommandHandler g_commandHandler(
    g_bus,
    g_modeManager,
    g_motionController,
    g_gpioManager,
    g_pwmManager,
    g_servoManager,
    g_stepperManager,
    g_telemetry,
    g_ultrasonicManager,
    g_encoder,
    g_dcMotorManager
);

// Modules
HeartbeatModule g_heartbeat(g_bus);
LoggingModule   g_logger(g_bus);
IdentityModule  g_identity(g_bus, g_multiTransport, "ESP32-bot");

// Loop schedulers (initialized in setup)
LoopScheduler g_safetyScheduler(20);   // Will be updated from LoopRates
LoopScheduler g_ctrlScheduler(20);     // Will be updated from LoopRates
LoopScheduler g_telemScheduler(100);   // Will be updated from LoopRates

// For periodic debug printing 
uint32_t g_lastIpPrintMs = 0;

ControlModule g_controlModule(
    &g_bus,
    &g_modeManager,
    &g_motionController,
    &g_encoder,
    &g_imu,
    &g_telemetry
);
ObserverManager& g_observers = g_controlModule.observers();

// Encoder / PID config for DC motor 0 
constexpr float ENCODER0_TICKS_PER_REV = 1632.67f;
static int32_t  g_lastTicks0  = 0;

// -----------------------------------------------------------------------------
// Forward declarations for helpers
// -----------------------------------------------------------------------------
void setupWifiDualMode();
void setupOta();
void setupTransportsAndRouter();
void setupTelemetryProviders();
void setupGpioAndMotors();
void setupSafety();
void updateLoopSchedulers();
void runControlLoop(uint32_t now_ms, float dt);
void runSafetyLoop(uint32_t now_ms);

// -----------------------------------------------------------------------------
// WiFi: AP + STA
// -----------------------------------------------------------------------------
void setupWifiDualMode() {
    Serial.println("[WiFi] Starting AP + optional STA...");

    WiFi.mode(WIFI_AP_STA);
    WiFi.persistent(false);
    WiFi.setAutoReconnect(false);
    WiFi.setAutoConnect(false);

    Serial.print("[WiFi][STA] Connecting to ");
    Serial.print(WIFI_STA_SSID);

    WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASSWORD);

    uint32_t start           = millis();
    const uint32_t timeoutMs = 10000;

    while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("[WiFi][STA] Connected, IP: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("[WiFi][STA] Failed; disabling STA to avoid reconnect spam.");
        WiFi.disconnect(true, true);
        WiFi.mode(WIFI_AP);
    }

    bool apOk = WiFi.softAP(AP_SSID, AP_PASS);
    if (apOk) {
        IPAddress apIp = WiFi.softAPIP();
        Serial.print("[WiFi][AP] Started, SSID: ");
        Serial.print(AP_SSID);
        Serial.print("  IP: ");
        Serial.println(apIp);
    } else {
        Serial.println("[WiFi][AP] Failed to start AP!");
    }

    g_wifi.begin();
}

// -----------------------------------------------------------------------------
// OTA setup
// -----------------------------------------------------------------------------
void setupOta() {
    ArduinoOTA.setHostname("ESP32-bot");

    ArduinoOTA
        .onStart([]() {
            String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
            Serial.println("[OTA] Start updating " + type);
        })
        .onEnd([]() {
            Serial.println("\n[OTA] End");
        })
        .onProgress([](unsigned int progress, unsigned int total) {
            Serial.printf("[OTA] Progress: %u%%\r", (progress * 100U) / total);
        })
        .onError([](ota_error_t error) {
            Serial.printf("[OTA] Error[%u]: ", error);
            if (error == OTA_AUTH_ERROR)         Serial.println("Auth Failed");
            else if (error == OTA_BEGIN_ERROR)   Serial.println("Begin Failed");
            else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
            else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
            else if (error == OTA_END_ERROR)     Serial.println("End Failed");
        });

    ArduinoOTA.begin();
    Serial.println("[OTA] Ready. You can now upload OTA as 'ESP32-bot.local'");
}

// -----------------------------------------------------------------------------
// Safety system setup
// -----------------------------------------------------------------------------
void setupSafety() {
    SafetyConfig config;
    config.host_timeout_ms   = 2000;
    config.motion_timeout_ms = 2000;
    config.max_linear_vel    = 0.5f;
    config.max_angular_vel   = 2.0f;
    
    config.estop_pin  = -1;  // TODO: Wire physical E-stop button
    config.bypass_pin = -1;  // TODO: Wire bypass switch
    config.relay_pin  = -1;  // TODO: Wire motor power relay

    g_modeManager.configure(config);
    g_modeManager.begin();

    g_modeManager.onStop([]() {
        Serial.println("[SAFETY] Stop triggered!");
        g_motionController.stop();
        g_dcMotorManager.stopAll();
    });

    Serial.println("[SAFETY] ModeManager configured and started");
}

// -----------------------------------------------------------------------------
// Transports, router, host wiring
// -----------------------------------------------------------------------------
void setupTransportsAndRouter() {
    g_multiTransport.addTransport(&g_uart);
    g_multiTransport.addTransport(&g_wifi);
#if HAS_BLE
    g_multiTransport.addTransport(&g_ble);
#endif

    g_commandHandler.setup();
    g_router.setup();

    g_host.setRouterLoop([&]() {
        g_router.loop();
    });

    g_host.addModule(&g_heartbeat);
    g_host.addModule(&g_logger);
    g_host.addModule(&g_identity);

    g_host.setup();
}

// -----------------------------------------------------------------------------
// Telemetry providers
// -----------------------------------------------------------------------------
void setupTelemetryProviders() {
    g_telemetry.setInterval(0);

    // Safety/Mode state
    g_telemetry.registerProvider(
        "mode",
        [&](ArduinoJson::JsonObject node) {
            node["state"]     = robotModeToString(g_modeManager.mode());
            node["can_move"]  = g_modeManager.canMove();
            node["estopped"]  = g_modeManager.isEstopped();
            node["connected"] = g_modeManager.isConnected();
            node["bypassed"]  = g_modeManager.isBypassed();
        }
    );

    // Loop rates
    g_telemetry.registerProvider(
        "rates",
        [&](ArduinoJson::JsonObject node) {
            LoopRates& r = getLoopRates();
            node["ctrl_hz"]   = r.ctrl_hz;
            node["safety_hz"] = r.safety_hz;
            node["telem_hz"]  = r.telem_hz;
        }
    );

    // Ultrasonic
    g_telemetry.registerProvider(
        "ultrasonic",
        [&](ArduinoJson::JsonObject node) {
            if (!g_ultrasonicManager.isAttached(0)) {
                node["sensor_id"] = 0;
                node["attached"]  = false;
                return;
            }
            float dist_cm = g_ultrasonicManager.readDistanceCm(0);
            node["sensor_id"]   = 0;
            node["attached"]    = true;
            node["ok"]          = (dist_cm >= 0.0f);
            node["distance_cm"] = (dist_cm >= 0.0f) ? dist_cm : -1.0f;
        }
    );

    // IMU
    g_telemetry.registerProvider(
        "imu",
        [&](ArduinoJson::JsonObject node) {
            node["online"] = g_imu.isOnline();
            if (!g_imu.isOnline()) {
                node["ok"] = false;
                return;
            }
            ImuManager::Sample s;
            bool ok = g_imu.readSample(s);
            node["ok"] = ok;
            if (!ok) return;
            node["ax_g"]   = s.ax_g;
            node["ay_g"]   = s.ay_g;
            node["az_g"]   = s.az_g;
            node["gx_dps"] = s.gx_dps;
            node["gy_dps"] = s.gy_dps;
            node["gz_dps"] = s.gz_dps;
            node["temp_c"] = s.temp_c;
        }
    );

    // LiDAR
    g_telemetry.registerProvider(
        "lidar",
        [&](ArduinoJson::JsonObject node) {
            node["online"] = g_lidar.isOnline();
            if (!g_lidar.isOnline()) {
                node["ok"] = false;
                return;
            }
            LidarManager::Sample s;
            bool ok = g_lidar.readSample(s);
            node["ok"] = ok;
            if (!ok) return;
            node["distance_m"] = s.distance_m;
        }
    );

    // Encoder 0
    g_telemetry.registerProvider(
        "encoder0",
        [&](ArduinoJson::JsonObject node) {
            int32_t ticks = g_encoder.getCount(0);
            node["ticks"] = ticks;
        }
    );

    // Stepper 0
    g_telemetry.registerProvider(
        "stepper0",
        [&](ArduinoJson::JsonObject node) {
            StepperManager::StepperDebugInfo info;
            if (!g_stepperManager.getStepperDebugInfo(0, info)) {
                node["motor_id"] = 0;
                node["attached"] = false;
                return;
            }
            node["motor_id"]        = info.motorId;
            node["attached"]        = info.attached;
            node["enabled"]         = info.enabled;
            node["moving"]          = info.moving;
            node["dir_forward"]     = info.lastDirForward;
            node["last_cmd_steps"]  = info.lastCmdSteps;
            node["last_cmd_speed"]  = info.lastCmdSpeed;
        }
    );

    // DC motor 0
    g_telemetry.registerProvider(
        "dc_motor0",
        [&](ArduinoJson::JsonObject node) {
            DcMotorManager::MotorDebugInfo info;
            if (!g_dcMotorManager.getMotorDebugInfo(0, info)) {
                node["motor_id"] = 0;
                node["attached"] = false;
                return;
            }
            node["motor_id"]        = info.id;
            node["attached"]        = info.attached;
            node["in1_pin"]         = info.in1Pin;
            node["in2_pin"]         = info.in2Pin;
            node["pwm_pin"]         = info.pwmPin;
            node["ledc_channel"]    = info.ledcChannel;
            node["speed"]           = info.lastSpeed;
            node["freq_hz"]         = info.freqHz;
            node["resolution_bits"] = info.resolution;
        }
    );

    g_telemetry.setup();
}

// -----------------------------------------------------------------------------
// GPIO + motors/steppers setup
// -----------------------------------------------------------------------------
void setupGpioAndMotors() {
    pinMode(Pins::LED_STATUS, OUTPUT);
    digitalWrite(Pins::LED_STATUS, LOW);

    for (size_t i = 0; i < GPIO_CHANNEL_COUNT; ++i) {
        const auto& def = GPIO_CHANNEL_DEFS[i];
        g_gpioManager.registerChannel(def.channel, def.pin, def.mode);
    }

    g_stepperManager.registerStepper(
        0,
        Pins::STEPPER0_STEP,
        Pins::STEPPER0_DIR,
        Pins::STEPPER0_EN,
        false
    );
    g_stepperManager.dumpAllStepperMappings();

    bool dcOk = g_dcMotorManager.attach(
        0,
        Pins::MOTOR_LEFT_IN1,
        Pins::MOTOR_LEFT_IN2,
        Pins::MOTOR_LEFT_PWM,
        0,
        15000,
        12
    );
    (void)dcOk;

    g_dcMotorManager.dumpAllMotorMappings();
}

// -----------------------------------------------------------------------------
// Update loop schedulers from LoopRates
// -----------------------------------------------------------------------------
void updateLoopSchedulers() {
    LoopRates& r = getLoopRates();
    g_safetyScheduler.setPeriodMs(r.safety_period_ms());
    g_ctrlScheduler.setPeriodMs(r.ctrl_period_ms());
    g_telemScheduler.setPeriodMs(r.telem_period_ms());
}

// -----------------------------------------------------------------------------
// Control loop (motion, PID, etc.)
// -----------------------------------------------------------------------------
void runControlLoop(uint32_t now_ms, float dt) {
    // DC motor 0 velocity PID (encoder -> omega -> PID -> PWM)
    static int32_t lastTicks = 0;
    
    int32_t ticks = g_encoder.getCount(0);
    int32_t deltaTicks = ticks - lastTicks;
    lastTicks = ticks;

    if (dt > 0.0f) {
        float revs = deltaTicks / ENCODER0_TICKS_PER_REV;
        float omega_rad_s = revs * 2.0f * PI / dt;
        g_dcMotorManager.updateVelocityPid(0, omega_rad_s, dt);
    }

    // Motion controller update (only if allowed)
    if (g_modeManager.canMove()) {
        g_motionController.update(dt);
    }
}

// -----------------------------------------------------------------------------
// Safety loop
// -----------------------------------------------------------------------------
void runSafetyLoop(uint32_t now_ms) {
    g_modeManager.update(now_ms);
}

// -----------------------------------------------------------------------------
// setup()
// -----------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n[MCU] Booting with UART1 + WiFi (AP+STA)...");

    SerialPort.begin(
        115200,
        SERIAL_8N1,
        Pins::UART1_RX,
        Pins::UART1_TX
    );

    setupWifiDualMode();
    setupOta();
    setupSafety();

    bool imuOk   = g_imu.begin(Pins::I2C_SDA, Pins::I2C_SCL, 0x68);
    bool lidarOk = g_lidar.begin(Pins::I2C_SDA, Pins::I2C_SCL);
    Serial.printf("[MCU] IMU init: %s\n",   imuOk   ? "OK" : "FAILED");
    Serial.printf("[MCU] LiDAR init: %s\n", lidarOk ? "OK" : "FAILED");

    setupTransportsAndRouter();
    setupGpioAndMotors();
    setupTelemetryProviders();
    g_commandHandler.setControlModule(&g_controlModule);
    g_host.addModule(&g_controlModule);


    // Initialize loop schedulers from LoopRates
    updateLoopSchedulers();

    Serial.println("[MCU] Setup complete. Waiting for host connection...");
    
    // Print initial rates
    LoopRates& r = getLoopRates();
    Serial.printf("[MCU] Loop rates: ctrl=%dHz safety=%dHz telem=%dHz\n",
                  r.ctrl_hz, r.safety_hz, r.telem_hz);
}

// -----------------------------------------------------------------------------
// loop()
// -----------------------------------------------------------------------------
void loop() {
    static uint32_t last_ms = millis();
    uint32_t now_ms = millis();
    float dt = (now_ms - last_ms) / 1000.0f;
    last_ms = now_ms;

    // OTA
    ArduinoOTA.handle();

    // Update scheduler periods (in case rates changed via command)
    updateLoopSchedulers();

    // Rate-limited safety loop
    if (g_safetyScheduler.tick(now_ms)) {
        runSafetyLoop(now_ms);
    }

    // Rate-limited control loop
    if (g_ctrlScheduler.tick(now_ms)) {
        float ctrl_dt = getLoopRates().ctrl_period_ms() / 1000.0f;
        runControlLoop(now_ms, ctrl_dt);
    }

    // Rate-limited telemetry
    if (g_telemScheduler.tick(now_ms)) {
        g_telemetry.loop(now_ms);
    }

    // Host + router + transports (always run)
    g_host.loop(now_ms);
    g_wifi.loop();
#if HAS_BLE
    g_ble.loop();
#endif

    // Minimal delay to prevent watchdog issues
    delay(1);
}
