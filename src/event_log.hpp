#pragma once
#include <vector>
#include <string>

#include "foxglove_data_loader/data_loader.hpp"

constexpr int64_t REACHED_EOF = -1;
constexpr int64_t UNEXPECTED_EOF = -2;
constexpr int64_t MALFORMED_EVENT = -3;

struct LCMEvent
{
    uint64_t event_number;
    uint64_t timestamp_us;
    std::string channel;
    std::vector<uint8_t> data;
};

int64_t read_next(const uint8_t *buf, size_t len, LCMEvent *event);
