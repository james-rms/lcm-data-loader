
#include "event_log.hpp"

const uint32_t SYNC_WORD = 0xEDA1DA01;

uint32_t decode_u32(const uint8_t *data)
{
    return uint64_t(data[3]) + (uint64_t(data[2]) << 8) + (uint64_t(data[1]) << 16) + (uint64_t(data[0]) << 24);
}

uint64_t decode_u64(const uint8_t *data)
{
    return uint64_t(data[7]) + (uint64_t(data[6]) << 8) + (uint64_t(data[5]) << 16) + (uint64_t(data[4]) << 24) + (uint64_t(data[3]) << 32) + (uint64_t(data[2]) << 40) + (uint64_t(data[1]) << 48) + (uint64_t(data[0]) << 56);
}

int64_t read_next(const uint8_t *buf, size_t len, LCMEvent *event)
{
    if (len == 0)
    {
        return 0;
    }
    if (len < 4 + 8 + 8 + 8)
    {
        return UNEXPECTED_EOF;
    }
    size_t cursor = 0;
    if (decode_u32(buf) != SYNC_WORD)
    {
        return MALFORMED_EVENT;
    }
    cursor += 4;
    event->event_number = decode_u64(&buf[cursor]);
    cursor += 8;
    event->timestamp_us = decode_u64(&buf[cursor]);
    cursor += 8;
    uint32_t channel_len = decode_u32(&buf[cursor]);
    cursor += 4;
    uint32_t data_len = decode_u32(&buf[cursor]);
    cursor += 4;

    const uint8_t *channel_start = buf + cursor;
    const uint8_t *data_start = buf + cursor + channel_len;

    // copy channel into channel string
    event->channel = std::string(reinterpret_cast<const char *>(channel_start), channel_len);

    // copy data into data vector
    event->data.resize(data_len);
    memcpy(event->data.data(), data_start, data_len);
    return int64_t(cursor + channel_len + data_len);
}
