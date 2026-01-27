#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>

#include "core/EventBus.h"
#include "core/MCUHost.h"
#include "core/MultiTransport.h"
#include "core/UartTransport.h"
#include "core/WifiTransport.h"
#include "core/MessageRouter.h"
#include "modules/HearbeatModule.h"
#include "modules/LoggingModule.h"
#include "config/WifiSecrets.h"
#include "core/BleTransport.h"
#include "modules/IdentityModule.h"
#include "core/ModeManager.h"
#include "core/CommandHandler.h"
#include "core/MotionController.h"
#include "config/PinConfig.h"
#include "managers/GpioManager.h"
#include "managers/PwmManager.h"
#include "managers/ServoManager.h"
#include "managers/StepperManager.h"
#include "managers/DcMotorManager.h"
#include "config/GpioChannelDefs.h"
#include "modules/TelemetryModule.h"
#include "managers/UltrasonicManager.h"
#include "managers/ImuManager.h"
#include "managers/LidarManager.h"
#include "managers/EncoderManager.h"

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

// Core bus + mode (SafetyManager removed - now part of ModeManager)
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

// Dedicated UART1 for protocol (note: UartTransport currently bound to Serial)
HardwareSerial SerialPort(1);          // UART1
UartTransport  g_uart(Serial, 115200); // protocol over USB Serial (for now)
//UartTransport g_uart(SerialPort, 115200);

WifiTransport  g_wifi(3333);           // TCP port
BleTransport   g_ble("ESP32-SPP");

// Router + host
MessageRouter g_router(g_bus, g_multiTransport);
MCUHost       g_host(g_bus, &g_router);

// Telemetry
TelemetryModule g_telemetry(g_bus);

// Command handler (SafetyManager removed from constructor)
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

// For periodic debug printing 
uint32_t g_lastIpPrintMs = 0;

// Encoder / PID config for DC motor 0 
constexpr float ENCODER0_TICKS_PER_REV = 1632.67f;  // without any attachments change when necesary
static uint32_t g_lastPidMs   = 0;
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
void updateDcMotor0VelocityPid(uint32_t now_ms);

// -----------------------------------------------------------------------------
// WiFi: AP + STA
// -----------------------------------------------------------------------------
void setupWifiDualMode() {
    Serial.println("[WiFi] Starting AP + optional STA...");

    WiFi.mode(WIFI_AP_STA);

    // Turn off persistent + reconnect BEFORE begin
    WiFi.persistent(false);
    WiFi.setAutoReconnect(false);
    WiFi.setAutoConnect(false);  // optional, prevents auto-connect at next boot

    // --- Try STA once ---
    Serial.print("[WiFi][STA] Connecting to ");
    Serial.print(WIFI_STA_SSID);

    WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASSWORD);

    uint32_t start           = millis();
    const uint32_t timeoutMs = 10000; // 10s timeout

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

        // Kill STA cleanly so it stops scanning / reconnecting
        WiFi.disconnect(true, true);   // drop + clear creds
        WiFi.mode(WIFI_AP);           // AP only from here
    }

    // --- Start robot AP ---
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
            String type;
            if (ArduinoOTA.getCommand() == U_FLASH) {
                type = "sketch";
            } else {
                type = "filesystem";
            }
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
// Safety system setup (ModeManager now handles this)
// -----------------------------------------------------------------------------
void setupSafety() {
    SafetyConfig config;
    config.host_timeout_ms   = 2000;
    config.motion_timeout_ms = 2000;
    config.max_linear_vel    = 0.5f;   // Match motion controller
    config.max_angular_vel   = 2.0f;   // Match motion controller
    
    // Set to -1 if pins not wired yet, or use actual pins
    config.estop_pin  = -1;  // TODO: Wire physical E-stop button
    config.bypass_pin = -1;  // TODO: Wire bypass switch for bench work
    config.relay_pin  = -1;  // TODO: Wire motor power relay

    g_modeManager.configure(config);
    g_modeManager.begin();

    // Register stop callback - called when safety triggers
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
    // Compose transports: UART1 + WiFi + BLE
    g_multiTransport.addTransport(&g_uart);   // binary on Serial/USB (or UART1 later)
    g_multiTransport.addTransport(&g_wifi);
    g_multiTransport.addTransport(&g_ble);    // this doesnt work for some reason yet

    // CommandHandler subscribes to JSON_MESSAGE_RX
    g_commandHandler.setup();

    // Router sets frame handler and begins transports
    g_router.setup();

    // Hook router loop into MCUHost
    g_host.setRouterLoop([&]() {
        g_router.loop();
    });

    // Register host modules
    g_host.addModule(&g_heartbeat);
    g_host.addModule(&g_logger);
    g_host.addModule(&g_identity);

    // Setup host (modules, timers, etc.)
    g_host.setup();
}

// -----------------------------------------------------------------------------
// Telemetry providers
// -----------------------------------------------------------------------------
void setupTelemetryProviders() {
    // Configure telemetry base interval (0 = off, >0 for periodic)
    g_telemetry.setInterval(0);  // you can bump this from the host

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
            node["gpio_ch_in1"]     = info.gpioChIn1;
            node["gpio_ch_in2"]     = info.gpioChIn2;
            node["pwm_ch"]          = info.pwmCh;
            node["speed"]           = info.lastSpeed;  // -1..1
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
    // Status LED
    pinMode(Pins::LED_STATUS, OUTPUT);
    digitalWrite(Pins::LED_STATUS, LOW);

    // GPIO logical channel mappings
    for (size_t i = 0; i < GPIO_CHANNEL_COUNT; ++i) {
        const auto& def = GPIO_CHANNEL_DEFS[i];
        g_gpioManager.registerChannel(def.channel, def.pin, def.mode);
    }

    // Stepper registration
    g_stepperManager.registerStepper(
        /*motorId   */ 0,
        /*pinStep   */ Pins::STEPPER0_STEP,
        /*pinDir    */ Pins::STEPPER0_DIR,
        /*pinEnable */ Pins::STEPPER0_EN,
        /*invertDir */ false
    );
    g_stepperManager.dumpAllStepperMappings();

    // DC motor attach (motor 0)
    bool dcOk = g_dcMotorManager.attach(
        /*id=*/0,
        Pins::MOTOR_LEFT_IN1,   // IN1
        Pins::MOTOR_LEFT_IN2,   // IN2
        Pins::MOTOR_LEFT_PWM,   // ENA (PWM)
        /*ledcChannel=*/0,      // hardware LEDC channel
        /*freq=*/15000,
        /*resolutionBits=*/12
    );
    (void)dcOk; // if you want, you can log this

    g_dcMotorManager.dumpAllMotorMappings();
}

// -----------------------------------------------------------------------------
// DC motor 0 velocity PID update helper
// -----------------------------------------------------------------------------
void updateDcMotor0VelocityPid(uint32_t now_ms) {
    const uint32_t PID_PERIOD_MS = 2;  // ~500 Hz

    if (now_ms - g_lastPidMs < PID_PERIOD_MS) {
        return;
    }

    uint32_t elapsed = now_ms - g_lastPidMs;
    g_lastPidMs = now_ms;

    float dt_pid = elapsed / 1000.0f;
    if (dt_pid <= 0.0f) return;

    int32_t ticks = g_encoder.getCount(0);
    int32_t deltaTicks = ticks - g_lastTicks0;
    g_lastTicks0 = ticks;

    // ticks -> revs -> rad/s
    float revs        = deltaTicks / ENCODER0_TICKS_PER_REV;
    float omega_rad_s = revs * 2.0f * PI / dt_pid;

    // This will do nothing if PID is disabled for motor 0
    g_dcMotorManager.updateVelocityPid(0, omega_rad_s, dt_pid);
}

// -----------------------------------------------------------------------------
// setup() and loop()
// -----------------------------------------------------------------------------
void setup() {
    // USB Serial for logs
    Serial.begin(115200);
    delay(500);
    Serial.println("\n[MCU] Booting with UART1 + WiFi (AP+STA)...");

    // UART1 for protocol (pins from PinConfig)
    SerialPort.begin(
        115200,
        SERIAL_8N1,
        Pins::UART1_RX,
        Pins::UART1_TX
    );

    setupWifiDualMode();
    setupOta();

    // Safety system (must be before transports so callbacks are ready)
    setupSafety();

    // Sensors
    bool imuOk   = g_imu.begin(Pins::I2C_SDA, Pins::I2C_SCL, 0x68);
    bool lidarOk = g_lidar.begin(Pins::I2C_SDA, Pins::I2C_SCL);
    Serial.printf("[MCU] IMU init: %s\n",   imuOk   ? "OK" : "FAILED");
    Serial.printf("[MCU] LiDAR init: %s\n", lidarOk ? "OK" : "FAILED");

    setupTransportsAndRouter();
    setupGpioAndMotors();
    setupTelemetryProviders();

    Serial.println("[MCU] Setup complete. Waiting for host connection...");
}

void loop() {
    static uint32_t last_ms = millis();
    uint32_t now_ms = millis();
    float dt = (now_ms - last_ms) / 1000.0f;
    last_ms = now_ms;

    // OTA
    ArduinoOTA.handle();

    // Safety system update (watchdogs, hardware inputs)
    g_modeManager.update(now_ms);

    // Host + router + transports
    g_host.loop(now_ms);
    g_wifi.loop();
    g_ble.loop();

    // Telemetry (periodic JSON -> host)
    g_telemetry.loop(now_ms);

    // DC motor 0 velocity PID (encoder -> omega -> PID -> PWM)
    updateDcMotor0VelocityPid(now_ms);

    // Motion control (only if allowed by safety)
    if (g_modeManager.canMove()) {
        g_motionController.update(dt);
    }

    delay(1);
}
