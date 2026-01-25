#include <unity.h>
#include <string>

// ✅ NOTE: do NOT write "include/core/..."
// include/ is already on the include path.
#include "core/Messages.h"   // <-- adjust if your file name differs

static void test_messages_json_smoke_parse() {
    const std::string json = R"({
      "kind":"cmd",
      "type":"PING",
      "seq":123,
      "payload":{}
    })";

    JsonMessage msg;
    bool ok = parseJsonToMessage(json, msg);

    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_UINT32(123, msg.seq);
}

int main(int argc, char** argv) {
    (void)argc; (void)argv;
    UNITY_BEGIN();
    RUN_TEST(test_messages_json_smoke_parse);
    return UNITY_END();
}
