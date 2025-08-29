#include "transcode.hpp"

#include "lcm/lcmtypes_velodyne_t.h"
#include "lcm/lcmtypes_laser_t.h"
#include "lcm/velodyne.h"
#include "lcm/lcmtypes_image_t.h"

#include <foxglove/schemas.hpp>

Transcoder::Transcoder()
{
    velodyne_calibration = velodyne_calib_create();
}

Transcoder::~Transcoder()
{
    free(velodyne_calibration);
}

template <typename T>
foxglove::FoxgloveError encode_to_vec(T &msg, std::vector<uint8_t> *out)
{
    size_t encoded_len = 0;
    foxglove::FoxgloveError error = msg.encode(out->data(), out->size(), &encoded_len);
    out->resize(encoded_len);
    if (error == foxglove::FoxgloveError::BufferTooShort)
    {
        error = msg.encode(out->data(), out->size(), &encoded_len);
    }
    return error;
}

int32_t Transcoder::transcode_point_cloud(const std::vector<uint8_t> &in, std::vector<uint8_t> *out, const char *frame_id)
{
    foxglove::schemas::PointCloud pointcloud;
    pointcloud.fields.push_back(foxglove::schemas::PackedElementField{.name = "x", .offset = 0, .type = foxglove::schemas::PackedElementField::NumericType::FLOAT64});
    pointcloud.fields.push_back(foxglove::schemas::PackedElementField{.name = "y", .offset = 8, .type = foxglove::schemas::PackedElementField::NumericType::FLOAT64});
    pointcloud.fields.push_back(foxglove::schemas::PackedElementField{.name = "z", .offset = 16, .type = foxglove::schemas::PackedElementField::NumericType::FLOAT64});
    pointcloud.fields.push_back(foxglove::schemas::PackedElementField{.name = "intensity", .offset = 24, .type = foxglove::schemas::PackedElementField::NumericType::FLOAT64});
    pointcloud.frame_id = frame_id;
    pointcloud.point_stride = 32;
    pointcloud.pose = foxglove::schemas::Pose{
        .orientation = foxglove::schemas::Quaternion{.w = 1},
    };

    lcmtypes_velodyne_t vel;
    lcmtypes_velodyne_t_decode(in.data(), 0, in.size(), &vel);
    pointcloud.timestamp.emplace(foxglove::schemas::Timestamp{.sec = uint32_t(vel.utime) / 1000000, .nsec = (uint32_t(vel.utime) % 1000000) * 1000});
    // parse the velodyne data packet
    velodyne_decoder_t vdecoder;
    velodyne_decoder_init(velodyne_calibration, &vdecoder, vel.data, vel.datalen);
    velodyne_sample_t vsample;

    while (!velodyne_decoder_next(velodyne_calibration, &vdecoder, &vsample))
    {
        if (vsample.range < 0.01)
        {
            continue;
        }
        constexpr size_t size = sizeof(double) * 3;
        std::byte bytes[size];
        std::memcpy(bytes, &vsample.xyz, size);
        pointcloud.data.insert(pointcloud.data.end(), bytes, bytes + size);
    }
    encode_to_vec(pointcloud, out);
    lcmtypes_velodyne_t_decode_cleanup(&vel);
    return 0;
}

int32_t Transcoder::transcode_laser_scan(const std::vector<uint8_t> &in, std::vector<uint8_t> *out, const char *frame_id)
{
    lcmtypes_laser_t msg;
    lcmtypes_laser_t_decode(in.data(), 0, in.size(), &msg);
    foxglove::schemas::LaserScan scan;
    scan.timestamp.emplace(foxglove::schemas::Timestamp{.sec = uint32_t(msg.utime) / 1000000, .nsec = (uint32_t(msg.utime) % 1000000) * 1000});
    scan.frame_id = frame_id;
    scan.pose = foxglove::schemas::Pose{
        .orientation = foxglove::schemas::Quaternion{.w = 1},
    };
    scan.start_angle = msg.rad0;
    scan.end_angle = msg.rad0 + msg.radstep * msg.nranges;
    std::vector<double> ranges(msg.nranges);
    for (int i = 0; i < msg.nranges; i++)
    {
        ranges[i] = msg.ranges[i];
    }
    std::vector<double> intensities(msg.nintensities);
    for (int i = 0; i < msg.nintensities; i++)
    {
        intensities[i] = msg.intensities[i];
    }
    scan.ranges = ranges;
    scan.intensities = intensities;
    encode_to_vec(scan, out);
    lcmtypes_laser_t_decode_cleanup(&msg);
    return 0;
}
int32_t Transcoder::transcode_image(const std::vector<uint8_t> &in, std::vector<uint8_t> *out, const char *frame_id)
{
    lcmtypes_image_t msg;
    lcmtypes_image_t_decode(in.data(), 0, in.size(), &msg);
    foxglove::schemas::CompressedImage img;
    uint32_t usec = uint32_t(msg.utime);
    img.timestamp.emplace(foxglove::schemas::Timestamp{.sec = usec / 1000000, .nsec = (usec % 1000000) * 1000});
    img.frame_id = frame_id;
    const std::byte* ptr = (const std::byte*)msg.image;
    img.data.insert(img.data.end(), ptr, ptr + msg.size);
    img.format = "jpeg";
    encode_to_vec(img, out);
    lcmtypes_image_t_decode_cleanup(&msg);
    return 0;
}
