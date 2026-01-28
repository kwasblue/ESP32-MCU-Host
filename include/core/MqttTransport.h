// include/core/MqttTransport.h

#pragma once

#include <WiFi.h>
#include <PubSubClient.h>
#include "core/ITransport.h"
#include <string>
#include <functional>

class MqttTransport : public ITransport {
public:
    using FrameCallback = std::function<void(const uint8_t*, size_t)>;
    
    MqttTransport(
        const char* broker,
        uint16_t port,
        const char* robotId,
        const char* username = nullptr,
        const char* password = nullptr
    );
    
    void begin() override;
    void loop() override;
    
    // FIX: Match ITransport return type (bool, not void)
    bool sendBytes(const uint8_t* data, size_t len) override;
    
    void setFrameHandler(FrameCallback cb) { frameCallback_ = cb; }
    
    // FIX: Remove const since PubSubClient::connected() is not const
    bool isConnected() { return mqtt_.connected(); }
    
private:
    void reconnect();
    void onMessage(char* topic, uint8_t* payload, unsigned int length);
    void publishDiscoveryResponse();
    
    WiFiClient wifi_;
    PubSubClient mqtt_;
    
    std::string broker_;
    uint16_t port_;
    std::string robotId_;
    std::string username_;
    std::string password_;
    
    // Topics
    std::string topicCmd_;
    std::string topicAck_;
    std::string topicTelemetry_;
    std::string topicState_;
    std::string topicDiscovery_;
    
    FrameCallback frameCallback_;
    
    uint32_t lastReconnectAttempt_ = 0;
    static constexpr uint32_t RECONNECT_INTERVAL_MS = 5000;
};