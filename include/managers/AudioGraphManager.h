#pragma once
#include <atomic>
#include <memory>

#include <ArduinoJson.h>

#include "core/EventBus.h"
#include "core/CommandHandler.h"

#include "audio/DspChain.h"
#include "audio/DspFactory.h"

class AudioGraphManager {
public:
    AudioGraphManager() = default;

    void attach(EventBus* bus, CommandHandler* cmd);
    void begin();

    void handleChainSet(const JsonObjectConst& msg);
    void handleChainGet(const JsonObjectConst& msg);

    audio::DspChain* activeChain() const { return active_chain_.load(); }

private:
    EventBus* bus_ = nullptr;
    CommandHandler* cmd_ = nullptr;

    std::atomic<audio::DspChain*> active_chain_{nullptr};

    void swapChain(std::unique_ptr<audio::DspChain> new_chain);
    bool validateChainMessage(const JsonObjectConst& msg);
};
