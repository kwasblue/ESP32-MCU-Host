// core/WifiTransport.h
#pragma once
#include <Arduino.h>
#include <WiFi.h>      // ESP32 WiFi
#include <vector>
#include "ITransport.h"
#include "Protocol.h"

class WifiTransport : public ITransport {
public:
    WifiTransport(uint16_t port)
        : server_(port) {}

    void begin() override {
        // Assumes WiFi already configured elsewhere (SSID/password)
        server_.begin();
        rxBuffer_.clear();
        rxBuffer_.reserve(256);
    }

    void loop() override {
        if (!client_ || !client_.connected()) {
            client_ = server_.available();
            return;
        }

        while (client_.available() > 0) {
            uint8_t b = static_cast<uint8_t>(client_.read());
            rxBuffer_.push_back(b);
        }

        if (handler_) {
            Protocol::extractFrames(rxBuffer_, [this](const uint8_t* frame, size_t len) {
                handler_(frame, len);
            });
        }
    }

    bool sendBytes(const uint8_t* data, size_t len) override {
        if (!client_ || !client_.connected()) return false;
        size_t written = client_.write(data, len);
        return written == len;
    }

private:
    WiFiServer          server_;
    WiFiClient          client_;
    std::vector<uint8_t> rxBuffer_;
};

// example code for setupwifi()

//#include <WiFi.h>
// void setupWifi() {
//     WiFi.mode(WIFI_STA);
//     WiFi.begin("YourHomeSSID", "YourHomePassword");
//     Serial.print("[WiFi] Connecting");
//     while (WiFi.status() != WL_CONNECTED) {
//         delay(500);
//         Serial.print(".");
//     }
//     Serial.println();
//     Serial.print("[WiFi] Connected, IP: ");
//     Serial.println(WiFi.localIP());
// }

// const char* WIFI_SSID = "RobotAP";
// const char* WIFI_PASS = "robotpass";  // change to whatever

// void setupWifi() {
//     WiFi.mode(WIFI_AP);
//     WiFi.softAP(WIFI_SSID, WIFI_PASS);
//     IPAddress ip = WiFi.softAPIP();
//     Serial.print("[WiFi] AP started, IP: ");
//     Serial.println(ip);
// }
 