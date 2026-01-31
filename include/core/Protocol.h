#pragma once
#include <cstdint>
#include <vector>
#include <functional>

namespace Protocol {

static constexpr uint8_t HEADER = 0xAA;

enum MsgType : uint8_t {
    MSG_HEARTBEAT        = 0x01,
    MSG_PING             = 0x02,
    MSG_PONG             = 0x03,
    MSG_VERSION_REQUEST  = 0x04,
    MSG_VERSION_RESPONSE = 0x05,
    MSG_WHOAMI           = 0x10,
    MSG_TELEMETRY_BIN    = 0x30,
    MSG_CMD_JSON         = 0x50,
    MSG_CMD_BIN          = 0x51,  // Binary command for high-rate streaming
};

// -----------------------------------------------------------------------------
// Encode: [HEADER][len_hi][len_lo][msgType][payload...][checksum]
// length = 1 + payloadLen  (bytes in [msgType][payload...])
// -----------------------------------------------------------------------------
inline void encode(uint8_t msgType,
                   const uint8_t* payload,
                   size_t payloadLen,
                   std::vector<uint8_t>& outFrame)
{
    outFrame.clear();

    uint16_t length = static_cast<uint16_t>(1 + payloadLen);

    uint8_t len_hi = static_cast<uint8_t>((length >> 8) & 0xFF);
    uint8_t len_lo = static_cast<uint8_t>(length & 0xFF);

    uint8_t checksum = static_cast<uint8_t>((length + msgType) & 0xFF);
    for (size_t i = 0; i < payloadLen; ++i) {
        checksum = static_cast<uint8_t>(checksum + payload[i]);
    }

    outFrame.reserve(1 + 2 + 1 + payloadLen + 1);

    outFrame.push_back(HEADER);
    outFrame.push_back(len_hi);
    outFrame.push_back(len_lo);
    outFrame.push_back(msgType);

    for (size_t i = 0; i < payloadLen; ++i) {
        outFrame.push_back(payload[i]);
    }

    outFrame.push_back(checksum);
}

// -----------------------------------------------------------------------------
// Decode: same format as above.
// Calls onFrame(body, length) where:
//     body[0] = msgType
//     body[1..length-1] = payload
// -----------------------------------------------------------------------------
inline void extractFrames(std::vector<uint8_t>& buffer,
                          const std::function<void(const uint8_t* frame, size_t len)>& onFrame)
{
    size_t i = 0;
    constexpr size_t MIN_FRAME_HEADER = 5;

    while (i + MIN_FRAME_HEADER <= buffer.size()) {
        if (buffer[i] != HEADER) {
            ++i;
            continue;
        }

        if (i + 3 > buffer.size()) {
            break;
        }

        uint8_t len_hi = buffer[i + 1];
        uint8_t len_lo = buffer[i + 2];
        uint16_t length = static_cast<uint16_t>((len_hi << 8) | len_lo);

        if (length < 1) {
            ++i;
            continue;
        }

        size_t frameTotal = 1 + 2 + static_cast<size_t>(length) + 1;

        if (i + frameTotal > buffer.size()) {
            break;
        }

        const uint8_t* body = &buffer[i + 3];
        uint8_t msgType = body[0];
        const uint8_t* payload = &body[1];
        size_t payloadLen = static_cast<size_t>(length) - 1;

        uint8_t recvChecksum = buffer[i + 3 + length];

        uint8_t checksum = static_cast<uint8_t>((length + msgType) & 0xFF);
        for (size_t j = 0; j < payloadLen; ++j) {
            checksum = static_cast<uint8_t>(checksum + payload[j]);
        }

        if (checksum == recvChecksum) {
            onFrame(body, static_cast<size_t>(length));
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