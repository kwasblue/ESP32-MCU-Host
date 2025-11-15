// core/BleTransport.h
#pragma once

#include <Arduino.h>
#include <BluetoothSerial.h>
#include <vector>

#include "ITransport.h"
#include "Protocol.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error "Bluetooth is not enabled! Enable it in menuconfig or via build flags."
#endif

class BleTransport : public ITransport {
public:
    explicit BleTransport(const char* deviceName)
        : name_(deviceName) {}

    void begin() override {
        if (!SerialBT_.begin(name_)) {
            Serial.println("[BleTransport] Failed to start BluetoothSerial");
            return;
        }
        Serial.print("[BleTransport] Started as: ");
        Serial.println(name_);

        rxBuffer_.clear();
        rxBuffer_.reserve(256);
    }

    void loop() override {
        // Read any bytes that arrived over Bluetooth SPP
        while (SerialBT_.available()) {
            uint8_t b = static_cast<uint8_t>(SerialBT_.read());
            rxBuffer_.push_back(b);
        }

        // Decode framed messages using the same Protocol as UART/TCP
        if (handler_) {
            Protocol::extractFrames(rxBuffer_, [this](const uint8_t* frame, size_t len) {
                // frame = [msg_type, payload...]
                handler_(frame, len);
            });
        }
    }

    bool sendBytes(const uint8_t* data, size_t len) override {
        if (!SerialBT_.hasClient()) {
            // No connected Bluetooth client; drop or just report false
            return false;
        }
        size_t written = SerialBT_.write(data, len);
        return written == len;
    }

private:
    const char*          name_;
    BluetoothSerial      SerialBT_;
    std::vector<uint8_t> rxBuffer_;
};
