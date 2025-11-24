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
#include "core/SafetyManager.h"
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

// Optional fallback defaults if WifiSecrets.h isn't set up yet
#ifndef WIFI_STA_SSID
#define WIFI_STA_SSID "YourHomeSSID"
#endif

#ifndef WIFI_STA_PASSWORD
#define WIFI_STA_PASSWORD "YourHomePassword"
#endif

// AP = robot's own network
const char* AP_SSID  = "RobotAP";
const char* AP_PASS  = "robotpass";

// === Globals ===

// Core bus + mode/safety
EventBus      g_bus;
ModeManager   g_modeManager; 
SafetyManager g_safetyManager;

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
HardwareSerial SerialPort(1);               // UART1
UartTransport  g_uart(Serial, 115200);      // protocol over UART1
 
WifiTransport  g_wifi(3333);                // TCP port
BleTransport   g_ble("ESP32-SPP");

// Router + host
MessageRouter g_router(g_bus, g_multiTransport);
MCUHost       g_host(g_bus, &g_router);

// Telemetry
TelemetryModule g_telemetry(g_bus);

// Command handler (JSON → mode/motion/safety/IO)
CommandHandler g_commandHandler(
    g_bus,
    g_modeManager,
    g_motionController,
    g_safetyManager,
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

// For periodic debug printing if you want later
uint32_t g_lastIpPrintMs = 0;

void setupWifiDualMode() {
    Serial.println("[WiFi] Starting AP + STA mode...");

    WiFi.mode(WIFI_AP_STA);

    // --- Connect to home network (STA mode) ---
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
        Serial.println("[WiFi][STA] Failed to connect (timeout). Continuing with AP only.");
    }

    // --- Start robot AP (AP mode) ---
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

    // 🔹 Start the WifiTransport TCP server here
    g_wifi.begin();   // <- this will print your WifiTransport debug lines
}

// === OTA setup ===
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

void setup() {
    // 1) USB Serial for logs
    Serial.begin(115200);
    delay(500);
    Serial.println("\n[MCU] Booting with UART1 + WiFi (AP+STA)...");

    // Servo calibration defaults (no attach yet; that can be done via SERVO_ATTACH)
    //g_servoManager.setScale(1.0f); // adjust scale of movement not sure if i really want this feature
    //g_servoManager.setOffset(0.0f);

    // 2) UART1 for protocol (binary frames on pins)
    SerialPort.begin(
        115200,
        SERIAL_8N1,
        Pins::UART1_RX,   // RX pin from PinConfig
        Pins::UART1_TX    // TX pin from PinConfig
    );

    // Bring up WiFi in dual mode (STA + AP)
    setupWifiDualMode();

    // Setup OTA
    setupOta();

    // Compose transports: UART1 + WiFi + BLE
    g_multiTransport.addTransport(&g_uart);   // binary on SerialPort (UART1)
    g_multiTransport.addTransport(&g_wifi);
    g_multiTransport.addTransport(&g_ble);    // ble diable for now

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


    // Configure telemetry rate (optional)
    g_telemetry.setInterval(0);  // 10 Hz enable this to have telemetry in your system
    // Initialize IMU
    bool imuOk = g_imu.begin(Pins::I2C_SDA, Pins::I2C_SCL, 0x68);
    Serial.printf("[MCU] IMU init: %s\n", imuOk ? "OK" : "FAILED");
         
    // Initialize LiDAR (VL53L0X on same I2C bus)
    bool lidarOk = g_lidar.begin(Pins::I2C_SDA, Pins::I2C_SCL);
    Serial.printf("[MCU] LiDAR init: %s\n", lidarOk ? "OK" : "FAILED");

    // example register providers
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
        // NEW: IMU telemetry provider
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
            if (!ok) {
                return;
            }

            // Accel in g
            node["ax_g"] = s.ax_g;
            node["ay_g"] = s.ay_g;
            node["az_g"] = s.az_g;

            // Gyro in deg/s
            node["gx_dps"] = s.gx_dps;
            node["gy_dps"] = s.gy_dps;
            node["gz_dps"] = s.gz_dps;

            // Temperature
            node["temp_c"] = s.temp_c;
        }
    );
        // NEW: LiDAR telemetry
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
            if (!ok) {
                return;
            }

            node["distance_m"] = s.distance_m;
        }
    );
    g_telemetry.registerProvider(
        "encoder0",
        [&](ArduinoJson::JsonObject node) {
            int32_t ticks = g_encoder.getCount(0);
            node["ticks"] = ticks;
            // add derived speed if you want
        }
    );
    g_telemetry.registerProvider(
        "stepper0",
        [&](ArduinoJson::JsonObject node) {
            StepperManager::StepperDebugInfo info;
            if (!g_stepperManager.getStepperDebugInfo(0, info)) {
                node["motor_id"] = 0;
                node["attached"] = false;
                return;
            }

            node["motor_id"] = info.motorId;
            node["attached"] = info.attached;

            node["enabled"]         = info.enabled;
            node["moving"]          = info.moving;
            node["dir_forward"]     = info.lastDirForward;
            node["last_cmd_steps"]  = info.lastCmdSteps;
            node["last_cmd_speed"]  = info.lastCmdSpeed;

            // If you want pins for debugging:
            // node["pin_step"]   = info.pinStep;
            // node["pin_dir"]    = info.pinDir;
            // node["pin_enable"] = info.pinEnable;
        }
    );

    g_telemetry.setup();


    // Setup host (modules, timers, etc.)
    g_host.setup();

    // Hardware stuff initial state
    pinMode(Pins::LED_STATUS, OUTPUT);
    digitalWrite(Pins::LED_STATUS, LOW);

    // === GPIO logical channel mappings ===
    for (size_t i = 0; i < GPIO_CHANNEL_COUNT; ++i) {
        const auto& def = GPIO_CHANNEL_DEFS[i];
        g_gpioManager.registerChannel(def.channel, def.pin, def.mode);
    }

    // === STEPPER REGISTRATION 
    g_stepperManager.registerStepper(
        /*motorId   */ 0,
        /*pinStep   */ Pins::STEPPER0_STEP,   // 19
        /*pinDir    */ Pins::STEPPER0_DIR,    // 23
        /*pinEnable */ Pins::STEPPER0_EN,     // 27
        /*invertDir */ false
    );
    g_stepperManager.dumpAllStepperMappings();

    // Attach motor 0 to your L298N pins (IN1, IN2, ENA/PWM)
    bool dcOk = g_dcMotorManager.attach(
        /*id=*/0,
        Pins::MOTOR_LEFT_IN1,   // IN1
        Pins::MOTOR_LEFT_IN2,   // IN2
        Pins::MOTOR_LEFT_PWM,   // ENA (PWM)
        /*ledcChannel=*/0,      // hardware LEDC channel
        /*freq=*/15000,         // 20 kHz
        /*resolutionBits=*/12   // <= key change: 10 bits instead of 12
    );

    g_dcMotorManager.dumpAllMotorMappings();

    if (!dcOk) {
        Serial.println("[MCU] DC motor attach FAILED");
    } else {
        Serial.println("[MCU] DC motor attach OK, running quick spin...");

        // Forward @ 40% for 2s
        g_dcMotorManager.setSpeed(0, +1.0f);
        delay(2000);

        // Stop 1s
        g_dcMotorManager.setSpeed(0, 0.0f);
        delay(1000);

        // Reverse @ 40% for 2s
        g_dcMotorManager.setSpeed(0, -1.0f);
        delay(2000);

        // Final stop
        g_dcMotorManager.setSpeed(0, 0.0f);
        Serial.println("[MCU] DC motor quick test done.");
    }


    Serial.println("[MCU] Setup complete.");
}

void loop() {
    static uint32_t last_ms = millis();
    uint32_t now_ms = millis();
    float dt = (now_ms - last_ms) / 1000.0f;
    last_ms = now_ms;

    // Handle OTA updates
    ArduinoOTA.handle();

    // Let host run its modules and router loop
    g_host.loop(now_ms);

    // start wifi manually
    g_wifi.loop();

    // start ble
    g_ble.loop();

    // Telemetry (periodic JSON -> host)
    g_telemetry.loop(now_ms);

    // Run motion control (diff-drive + servo interpolation)
    g_motionController.update(dt);

    delay(1);
}
