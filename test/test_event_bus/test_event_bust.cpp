#include <unity.h>
#include <vector>
#include <string>

#include "core/EventBus.h"
#include "core/Event.h"

static std::vector<std::string> calls;

static void handlerA(const Event&) { calls.push_back("A"); }
static void handlerB(const Event&) { calls.push_back("B"); }

void setUp() { calls.clear(); }
void tearDown() {}

void test_eventbus_calls_handlers_in_order() {
    EventBus bus;
    bus.subscribe(handlerA);
    bus.subscribe(handlerB);

    Event evt{};
    evt.type = EventType::PING;
    evt.timestamp_ms = 123;

    bus.publish(evt);

    TEST_ASSERT_EQUAL_INT(2, (int)calls.size());
    TEST_ASSERT_EQUAL_STRING("A", calls[0].c_str());
    TEST_ASSERT_EQUAL_STRING("B", calls[1].c_str());
}

void test_eventbus_ignores_null_handler() {
    EventBus bus;
    bus.subscribe(nullptr);
    bus.subscribe(handlerA);

    Event evt{};
    evt.type = EventType::PING;

    bus.publish(evt);

    TEST_ASSERT_EQUAL_INT(1, (int)calls.size());
    TEST_ASSERT_EQUAL_STRING("A", calls[0].c_str());
}

#ifdef ARDUINO
#include <Arduino.h>
void setup() {
    UNITY_BEGIN();
    RUN_TEST(test_eventbus_calls_handlers_in_order);
    RUN_TEST(test_eventbus_ignores_null_handler);
    UNITY_END();
}
void loop() {}
#else
int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_eventbus_calls_handlers_in_order);
    RUN_TEST(test_eventbus_ignores_null_handler);
    return UNITY_END();
}
#endif
