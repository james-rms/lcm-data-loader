#include "transcode.hpp"

#include "lcm/lcmtypes_velodyne_t.h"
#include "lcm/lcmtypes_laser_t.h"
#include "lcm/velodyne.h"
#include "lcm/lcmtypes_image_t.h"

#include "foxglove/PointCloud.pb-c.h"
#include "foxglove/LaserScan.pb-c.h"
#include "foxglove/CompressedImage.pb-c.h"

Transcoder::Transcoder()
{
    velodyne_calibration = velodyne_calib_create();
}

Transcoder::~Transcoder()
{
    free(velodyne_calibration);
}

int32_t Transcoder::transcode_point_cloud(const std::vector<uint8_t> &in, std::vector<uint8_t> *out, const char *frame_id)
{
    Foxglove__PointCloud pc;
    foxglove__point_cloud__init(&pc);
    Foxglove__PackedElementField x, y, z, intensity;
    foxglove__packed_element_field__init(&x);
    x.name = (char *)"x";
    x.offset = 0;
    x.type = FOXGLOVE__PACKED_ELEMENT_FIELD__NUMERIC_TYPE__FLOAT64;
    foxglove__packed_element_field__init(&y);
    y.name = (char *)"y";
    y.offset = 8;
    y.type = FOXGLOVE__PACKED_ELEMENT_FIELD__NUMERIC_TYPE__FLOAT64;
    foxglove__packed_element_field__init(&z);
    z.name = (char *)"z";
    z.offset = 16;
    z.type = FOXGLOVE__PACKED_ELEMENT_FIELD__NUMERIC_TYPE__FLOAT64;
    foxglove__packed_element_field__init(&intensity);
    intensity.name = (char *)"intensity";
    intensity.offset = 24;
    intensity.type = FOXGLOVE__PACKED_ELEMENT_FIELD__NUMERIC_TYPE__FLOAT64;
    pc.n_fields = 4;
    Foxglove__PackedElementField *field_ptrs[] = {&x, &y, &z, &intensity};
    pc.fields = field_ptrs;

    pc.frame_id = (char *)frame_id;
    pc.point_stride = 32;
    Foxglove__Pose pose;
    foxglove__pose__init(&pose);
    Foxglove__Quaternion orientation;
    foxglove__quaternion__init(&orientation);
    orientation.w = 1;
    Foxglove__Vector3 position;
    foxglove__vector3__init(&position);
    pose.orientation = &orientation;
    pose.position = &position;
    pc.pose = &pose;
    std::vector<uint8_t> data;

    lcmtypes_velodyne_t vel;
    lcmtypes_velodyne_t_decode(in.data(), 0, in.size(), &vel);
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
        uint8_t bytes[sizeof(double) * 3];
        std::memcpy(bytes, &vsample.xyz, sizeof(double) * 3);
        data.insert(data.end(), bytes, bytes + (sizeof(double) * 3));
    }
    pc.data.data = data.data();
    pc.data.len = data.size();

    size_t packed_size = foxglove__point_cloud__get_packed_size(&pc);
    out->resize(packed_size);
    foxglove__point_cloud__pack(&pc, out->data());
    lcmtypes_velodyne_t_decode_cleanup(&vel);
    return 0;
}

int32_t Transcoder::transcode_laser_scan(const std::vector<uint8_t> &in, std::vector<uint8_t> *out, const char *frame_id)
{
    lcmtypes_laser_t msg;
    lcmtypes_laser_t_decode(in.data(), 0, in.size(), &msg);
    Foxglove__LaserScan scan;
    foxglove__laser_scan__init(&scan);
    Google__Protobuf__Timestamp timestamp;
    google__protobuf__timestamp__init(&timestamp);
    uint64_t all_nanos = msg.utime * 1000;
    timestamp.nanos = all_nanos % 1000000000;
    timestamp.seconds = all_nanos / 1000000000;
    scan.timestamp = &timestamp;
    scan.frame_id = (char *)frame_id;
    Foxglove__Pose pose;
    foxglove__pose__init(&pose);
    Foxglove__Quaternion orientation;
    foxglove__quaternion__init(&orientation);
    orientation.w = 1;
    Foxglove__Vector3 position;
    foxglove__vector3__init(&position);
    position.x = 0;
    position.y = 0;
    position.z = 0;
    pose.position = &position;
    pose.orientation = &orientation;
    scan.pose = &pose;
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
    scan.ranges = ranges.data();
    scan.intensities = intensities.data();
    scan.n_ranges = msg.nranges;
    scan.n_intensities = msg.nintensities;
    size_t packed_size = foxglove__laser_scan__get_packed_size(&scan);
    out->resize(packed_size);
    foxglove__laser_scan__pack(&scan, out->data());
    lcmtypes_laser_t_decode_cleanup(&msg);
    return 0;
}
int32_t Transcoder::transcode_gps(const std::vector<uint8_t> &in, std::vector<uint8_t> *out, const char *frame_id) { return -1; }
int32_t Transcoder::transcode_image(const std::vector<uint8_t> &in, std::vector<uint8_t> *out, const char *frame_id)
{
    lcmtypes_image_t msg;
    lcmtypes_image_t_decode(in.data(), 0, in.size(), &msg);
    Foxglove__CompressedImage img;
    foxglove__compressed_image__init(&img);
    Google__Protobuf__Timestamp timestamp;
    google__protobuf__timestamp__init(&timestamp);
    uint64_t all_nanos = msg.utime * 1000;
    timestamp.nanos = all_nanos % 1000000000;
    timestamp.seconds = all_nanos / 1000000000;
    img.timestamp = &timestamp;
    img.frame_id = (char *)frame_id;
    img.data.data = msg.image;
    img.data.len = msg.size;
    img.format = (char *)"jpeg";
    size_t packed_size = foxglove__compressed_image__get_packed_size(&img);
    out->resize(packed_size);
    foxglove__compressed_image__pack(&img, out->data());
    lcmtypes_image_t_decode_cleanup(&msg);
    return 0;
}
int32_t Transcoder::transcode_poses_in_frame(const std::vector<uint8_t> &in, std::vector<uint8_t> *out, const char *frame_id) { return -1; }
