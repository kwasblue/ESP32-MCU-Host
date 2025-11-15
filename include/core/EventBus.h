#pragma once
#include <vector>
#include <functional>
#include "Event.h"

class EventBus {
public:
    using Handler = std::function<void(const Event&)>;

    void subscribe(const Handler& handler) {
        handlers_.push_back(handler);
    }

    void publish(const Event& evt) {
        for (auto& h : handlers_) {
            h(evt);
        }
    }

private:
    std::vector<Handler> handlers_;
};
