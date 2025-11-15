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

// === WiFi config ===
// STA = your home / main network
const char* STA_SSID = "YourHomeSSID";      // TODO: change this
const char* STA_PASS = "YourHomePassword";  // TODO: change this

// AP = robot's own network
const char* AP_SSID  = "RobotAP";
const char* AP_PASS  = "robotpass";

// Transports
EventBus        g_bus;
MultiTransport  g_multiTransport;
UartTransport   g_uart(Serial, 115200);
WifiTransport   g_wifi(3333);   // TCP port, listens on both AP + STA IPs
BleTransport    g_ble("ESP32-SPP"); 

MessageRouter   g_router(g_bus, g_multiTransport);
MCUHost         g_host(g_bus, &g_router);

// Modules
HeartbeatModule g_heartbeat(g_bus);
LoggingModule   g_logger(g_bus);
IdentityModule g_identity(g_bus, g_multiTransport, "kwasi-bot");

// For periodic debug printing
uint32_t g_lastIpPrintMs = 0;

void setupWifiDualMode() {
    Serial.println("[WiFi] Starting AP + STA mode...");
    
    // Enable both station and access point
    WiFi.mode(WIFI_AP_STA);

    // --- Connect to home network (STA mode) ---
    WiFi.begin(STA_SSID, STA_PASS);
    Serial.print("[WiFi][STA] Connecting to ");
    Serial.print(STA_SSID);
    WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASSWORD);

    uint32_t start = millis();
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
void setupOta() {
    // Use the same hostname everywhere
    ArduinoOTA.setHostname("ESP32-bot");

    // Optional: set a password if you want
    // ArduinoOTA.setPassword("someStrongPassword");

    ArduinoOTA
        .onStart([]() {
            String type;
            if (ArduinoOTA.getCommand() == U_FLASH) {
                type = "sketch";
            } else { // U_SPIFFS
                type = "filesystem";
            }
            Serial.println("[OTA] Start updating " + type);
        })
        .onEnd([]() {
            Serial.println("\n[OTA] End");
        })
        .onProgress([](unsigned int progress, unsigned int total) {
            Serial.printf("[OTA] Progress: %u%%\r", (progress * 100) / total);
        })
        .onError([](ota_error_t error) {
            Serial.printf("[OTA] Error[%u]: ", error);
            if (error == OTA_AUTH_ERROR)       Serial.println("Auth Failed");
            else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
            else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
            else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
            else if (error == OTA_END_ERROR)   Serial.println("End Failed");
        });

    ArduinoOTA.begin();
    Serial.println("[OTA] Ready. You can now upload OTA as 'kwasi-bot.local'");
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n[MCU] Booting with UART + WiFi (AP+STA)...");

    // Bring up WiFi in dual mode
    setupWifiDualMode();

    // setup OTA
    setupOta();  

    // Compose transports: UART + WiFi
    g_multiTransport.addTransport(&g_uart);
    g_multiTransport.addTransport(&g_wifi);
    g_multiTransport.addTransport(&g_ble);

    // Router + host + modules
    g_router.setup();
    g_host.setRouterLoop([&]() { g_router.loop(); });

    g_host.addModule(&g_heartbeat);
    g_host.addModule(&g_logger);
    g_host.addModule(&g_identity);

    g_host.setup();

    Serial.println("[MCU] Setup complete.");
}

void loop() {
    uint32_t now_ms = millis();
    
    // Handle OTA updates
    ArduinoOTA.handle();
    g_host.loop(now_ms);

    // Periodically print IPs so you always know how to connect
    //if (now_ms - g_lastIpPrintMs >= 5000) {
    //    g_lastIpPrintMs = now_ms;

        //Serial.print("[WiFi][STA] IP: ");
        //Serial.println(WiFi.localIP());    // 0.0.0.0 if not connected

        //Serial.print("[WiFi][AP ] IP: ");
        //Serial.println(WiFi.softAPIP());   // usually 192.168.4.1
    //}

    delay(1);
}
