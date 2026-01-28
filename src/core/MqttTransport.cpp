// src/core/MqttTransport.cpp

#include "core/MqttTransport.h"
#include "config/Version.h"  // Make sure this exists!
#include <ArduinoJson.h>

MqttTransport::MqttTransport(
    const char* broker,
    uint16_t port,
    const char* robotId,
    const char* username,
    const char* password
) : 
    mqtt_(wifi_),
    broker_(broker),
    port_(port),
    robotId_(robotId),
    username_(username ? username : ""),
    password_(password ? password : "")
{
    // Build topic strings
    topicCmd_       = "mara/" + robotId_ + "/cmd";
    topicAck_       = "mara/" + robotId_ + "/ack";
    topicTelemetry_ = "mara/" + robotId_ + "/telemetry";
    topicState_     = "mara/" + robotId_ + "/state";
    topicDiscovery_ = "mara/fleet/discover";
}

void MqttTransport::begin() {
    mqtt_.setServer(broker_.c_str(), port_);
    mqtt_.setBufferSize(1024);
    
    mqtt_.setCallback([this](char* topic, uint8_t* payload, unsigned int length) {
        this->onMessage(topic, payload, length);
    });
    
    reconnect();
}

void MqttTransport::loop() {
    if (!mqtt_.connected()) {
        uint32_t now = millis();
        if (now - lastReconnectAttempt_ > RECONNECT_INTERVAL_MS) {
            lastReconnectAttempt_ = now;
            reconnect();
        }
    } else {
        mqtt_.loop();
    }
}

void MqttTransport::reconnect() {
    if (mqtt_.connected()) return;
    
    Serial.printf("[MQTT] Connecting to %s:%d as %s...\n", 
        broker_.c_str(), port_, robotId_.c_str());
    
    bool connected = false;
    if (username_.empty()) {
        connected = mqtt_.connect(robotId_.c_str());
    } else {
        connected = mqtt_.connect(
            robotId_.c_str(), 
            username_.c_str(), 
            password_.c_str()
        );
    }
    
    if (connected) {
        Serial.println("[MQTT] Connected!");
        
        // Subscribe to command topic
        mqtt_.subscribe(topicCmd_.c_str());
        
        // Subscribe to fleet discovery
        mqtt_.subscribe(topicDiscovery_.c_str());
        
        // Announce presence
        publishDiscoveryResponse();
        
    } else {
        Serial.printf("[MQTT] Failed, rc=%d\n", mqtt_.state());
    }
}

void MqttTransport::onMessage(char* topic, uint8_t* payload, unsigned int length) {
    std::string t(topic);
    
    if (t == topicCmd_) {
        // Command from host
        if (frameCallback_) {
            frameCallback_(payload, length);
        }
    } else if (t == topicDiscovery_) {
        // Discovery request
        publishDiscoveryResponse();
    }
}

// FIX: Return bool instead of void
bool MqttTransport::sendBytes(const uint8_t* data, size_t len) {
    if (mqtt_.connected()) {
        return mqtt_.publish(topicAck_.c_str(), data, len);
    }
    return false;
}

void MqttTransport::publishDiscoveryResponse() {
    JsonDocument doc;
    doc["robot_id"] = robotId_;
    
    // FIX: Use the correct constant names from your Version.h
    // Check your config/Version.h for the actual names
    #ifdef MARA_FIRMWARE_VERSION
        doc["firmware"] = MARA_FIRMWARE_VERSION;
    #else
        doc["firmware"] = "1.0.0";  // Fallback
    #endif
    
    #ifdef MARA_PROTOCOL_VERSION
        doc["protocol"] = MARA_PROTOCOL_VERSION;
    #else
        doc["protocol"] = 1;  // Fallback
    #endif
    
    #ifdef MARA_BOARD_TYPE
        doc["board"] = MARA_BOARD_TYPE;
    #else
        doc["board"] = "esp32";  // Fallback
    #endif
    
    doc["state"] = "ONLINE";
    
    std::string json;
    serializeJson(doc, json);
    
    mqtt_.publish("mara/fleet/discover_response", json.c_str());
}