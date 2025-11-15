// core/MultiTransport.h
#pragma once
#include <vector>
#include "ITransport.h"

class MultiTransport : public ITransport {
public:
    void addTransport(ITransport* t) {
        transports_.push_back(t);
    }

    void begin() override {
        for (auto* t : transports_) {
            if (!t) continue;
            // For each underlying transport, wire its frame handler back to ours
            t->setFrameHandler([this](const uint8_t* data, size_t len) {
                // Any frame from any transport gets passed up as if it came from one
                if (this->handler_) {
                    this->handler_(data, len);
                }
            });
            t->begin();
        }
    }

    void loop() override {
        for (auto* t : transports_) {
            if (t) t->loop();
        }
    }

    bool sendBytes(const uint8_t* data, size_t len) override {
        bool ok = true;
        for (auto* t : transports_) {
            if (t) {
                ok = t->sendBytes(data, len) && ok;
            }
        }
        return ok;
    }

private:
    std::vector<ITransport*> transports_;
};
