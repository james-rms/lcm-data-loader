#pragma once
#include <vector>
#include <memory>
#include "lcm/velodyne.h"
#include "foxglove/PackedElementField.pb-c.h"

struct Transcoder
{
    velodyne_calib_t *velodyne_calibration;

    Transcoder();
    ~Transcoder();
    int32_t transcode_point_cloud(const std::vector<uint8_t> &in, std::vector<uint8_t> *out, const char *frame_id);
    int32_t transcode_laser_scan(const std::vector<uint8_t> &in, std::vector<uint8_t> *out, const char *frame_id);
    int32_t transcode_gps(const std::vector<uint8_t> &in, std::vector<uint8_t> *out, const char *frame_id);
    int32_t transcode_image(const std::vector<uint8_t> &in, std::vector<uint8_t> *out, const char *frame_id);
    int32_t transcode_poses_in_frame(const std::vector<uint8_t> &in, std::vector<uint8_t> *out, const char *frame_id);
};
