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
#include "config/GpioChannelDefs.h"

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

// Core bus + managers
EventBus         g_bus;
ModeManager      g_modeManager;
MotionController g_motionController;
SafetyManager    g_safetyManager;

// Transports
MultiTransport   g_multiTransport;

// Dedicated UART1 for protocol
HardwareSerial   SerialPort(1);              // UART1
UartTransport    g_uart(SerialPort, 115200); // protocol over UART1 pins

WifiTransport    g_wifi(3333);               // TCP port
BleTransport     g_ble("ESP32-SPP");

// Router + host
MessageRouter    g_router(g_bus, g_multiTransport);
MCUHost          g_host(g_bus, &g_router);

// Hardware managers
GpioManager    g_gpioManager;
PwmManager     g_pwmManager;
ServoManager   g_servoManager;
StepperManager g_stepperManager;

// Command handler (JSON → mode/motion/safety/IO)
CommandHandler   g_commandHandler(
    g_bus,
    g_modeManager,
    g_motionController,
    g_safetyManager,
    g_gpioManager,
    g_pwmManager,
    g_servoManager,
    g_stepperManager
);

// Modules
HeartbeatModule  g_heartbeat(g_bus);
LoggingModule    g_logger(g_bus);
IdentityModule   g_identity(g_bus, g_multiTransport, "ESP32-bot");

// For periodic debug printing
uint32_t g_lastIpPrintMs = 0;

// === WiFi setup: AP + STA mode ===
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
    g_servoManager.setScale(2.0f);   // if needed
    g_servoManager.setOffset(0.0f);  // tweak if you want to shift everything
    

    // 2) UART1 for protocol (binary frames on pins)
    SerialPort.begin(
        115200,
        SERIAL_8N1,
        Pins::UART1_RX,   // RX pin from pins.json
        Pins::UART1_TX    // TX pin from pins.json
    );

    // Bring up WiFi in dual mode (STA + AP)
    setupWifiDualMode();

    // Setup OTA
    setupOta();

    // Compose transports: UART1 + WiFi + BLE
    g_multiTransport.addTransport(&g_uart);   // binary on SerialPort (UART1)
    g_multiTransport.addTransport(&g_wifi);
    g_multiTransport.addTransport(&g_ble);

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

    Serial.println("[MCU] Setup complete.");

    // Hardware stuff initial state
    pinMode(Pins::LED_STATUS, OUTPUT);
    digitalWrite(Pins::LED_STATUS, LOW);


    // === GPIO logical channel mappings ===
    for (size_t i = 0; i < GPIO_CHANNEL_COUNT; ++i) {
        const auto& def = GPIO_CHANNEL_DEFS[i];
        g_gpioManager.registerChannel(def.channel, def.pin, def.mode);
    }
    
    // g_gpioManager.registerChannel(1, Pins::ULTRASONIC_TRIG, OUTPUT);
    // g_gpioManager.registerChannel(2, Pins::ULTRASONIC_ECHO, INPUT);
}

void loop() {
    uint32_t now_ms = millis();

    // Handle OTA updates
    ArduinoOTA.handle();

    // Let host run its modules and router loop
    g_host.loop(now_ms);

    delay(1);
}
