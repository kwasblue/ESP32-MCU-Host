#pragma once
#include <cstdint>
#include <vector>
#include <functional>

namespace Protocol {

static constexpr uint8_t HEADER = 0xAA;

enum MsgType : uint8_t {
    MSG_HEARTBEAT = 0x01,
    MSG_PING      = 0x02,
    MSG_PONG      = 0x03,
    MSG_WHOAMI    = 0x10
};

inline void encode(uint8_t msgType,
                   const uint8_t* payload,
                   size_t payloadLen,
                   std::vector<uint8_t>& outFrame)
{
    outFrame.clear();
    uint8_t len = static_cast<uint8_t>(1 + payloadLen); // msgType + payload

    uint8_t checksum = len + msgType;
    for (size_t i = 0; i < payloadLen; ++i) {
        checksum = static_cast<uint8_t>(checksum + payload[i]);
    }

    outFrame.reserve(3 + payloadLen);
    outFrame.push_back(HEADER);
    outFrame.push_back(len);
    outFrame.push_back(msgType);
    for (size_t i = 0; i < payloadLen; ++i) {
        outFrame.push_back(payload[i]);
    }
    outFrame.push_back(checksum);
}

inline void extractFrames(std::vector<uint8_t>& buffer,
                          const std::function<void(const uint8_t* frame, size_t len)>& onFrame)
{
    size_t i = 0;
    while (i + 3 <= buffer.size()) {
        if (buffer[i] != HEADER) {
            ++i;
            continue;
        }

        if (i + 2 >= buffer.size()) break;
        uint8_t len = buffer[i + 1];
        size_t frameTotal = 2 + len + 1; // header + len + [len bytes] + checksum
        if (i + frameTotal > buffer.size()) break;

        const uint8_t* body     = &buffer[i + 2];
        uint8_t        msgType  = body[0];
        const uint8_t* payload  = &body[1];
        size_t         payloadLen = len - 1;
        uint8_t        recvChecksum = buffer[i + 2 + len];

        uint8_t checksum = len + msgType;
        for (size_t j = 0; j < payloadLen; ++j) {
            checksum = static_cast<uint8_t>(checksum + payload[j]);
        }

        if (checksum == recvChecksum) {
            onFrame(body, len); // msgType + payload
            i += frameTotal;
        } else {
            ++i;
        }
    }

    if (i > 0) {
        buffer.erase(buffer.begin(), buffer.begin() + static_cast<long>(i));
    }
}

} // namespace Protocol
