#define FOXGLOVE_DATA_LOADER_IMPLEMENTATION
#include "foxglove_data_loader/data_loader.hpp"
#include "event_log.hpp"
#include "transcode.hpp"

#include "descriptors/CompressedImage.h"
#include "descriptors/LaserScan.h"
#include "descriptors/LocationFix.h"
#include "descriptors/PointCloud.h"
#include "descriptors/PosesInFrame.h"

#include <memory>
#include <sstream>

constexpr uint16_t CHANNEL_POSE = 0;
constexpr uint16_t CHANNEL_CAM_THUMB_RFR = 1;
constexpr uint16_t CHANNEL_CAM_THUMB_RFC = 2;
constexpr uint16_t CHANNEL_GPS_TO_LOCAL = 3;
constexpr uint16_t CHANNEL_VELODYNE = 4;
constexpr uint16_t CHANNEL_BROOM_L = 5;
constexpr uint16_t CHANNEL_BROOM_R = 6;
constexpr uint16_t CHANNEL_BROOM_C = 7;
constexpr uint16_t CHANNEL_BROOM_CL = 8;
constexpr uint16_t CHANNEL_BROOM_CR = 9;

constexpr uint16_t SCHEMA_POSES_IN_FRAME = 1;
constexpr uint16_t SCHEMA_COMPRESSED_IMAGE = 2;
constexpr uint16_t SCHEMA_LOCATION_FIX = 3;
constexpr uint16_t SCHEMA_POINT_CLOUD = 4;
constexpr uint16_t SCHEMA_LASER_SCAN = 5;

using namespace foxglove_data_loader;

std::string print_inner(std::stringstream &ss)
{
  return ss.str();
}

template <typename T, typename... Types>
std::string print_inner(std::stringstream &ss, T var1, Types... rest)
{
  ss << " " << var1;
  return print_inner(ss, rest...);
}

template <typename... Types>
void log(Types... vars)
{
  std::stringstream ss;
  std::string as_string = print_inner(ss, vars...);
  console_log(as_string.c_str());
}

template <typename... Types>
void warn(Types... vars)
{
  std::stringstream ss;
  std::string as_string = print_inner(ss, vars...);
  console_warn(as_string.c_str());
}

template <typename... Types>
void error(Types... vars)
{
  std::stringstream ss;
  std::string as_string = print_inner(ss, vars...);
  console_error(as_string.c_str());
}

struct EventIndex
{
  uint64_t offset;
  uint16_t channel_id;
  uint16_t schema_id;
  uint64_t timestamp_ns;
};

/** A simple data loader implementation that loads text files and yields each line as a message.
 * This data loader is initialized with a set of text files, which it reads into memory.
 * `create_iterator` returns an iterator which iterates over each file line-by-line, assigning
 * sequential timestamps starting from zero. Each line message uses its filename as its topic name.
 */
class LCMDataLoader : public foxglove_data_loader::AbstractDataLoader
{
public:
  std::vector<std::string> paths;
  std::vector<EventIndex> index;
  std::vector<uint8_t> buffer;
  LCMDataLoader(std::vector<std::string> paths);

  Result<Initialization> initialize() override;

  Result<std::unique_ptr<AbstractMessageIterator>> create_iterator(const MessageIteratorArgs &args) override;
};

/** Iterates over 'messages' that match the requested args. */
class LCMMessageIterator : public foxglove_data_loader::AbstractMessageIterator
{
  LCMDataLoader *data_loader;
  MessageIteratorArgs args;
  size_t index_pos;
  std::vector<uint8_t> last_serialized_message;
  Transcoder transcoder;
  LCMEvent current_event;

public:
  explicit LCMMessageIterator(LCMDataLoader *loader, MessageIteratorArgs args_);
  std::optional<Result<Message>> next() override;
};

LCMDataLoader::LCMDataLoader(std::vector<std::string> paths)
{
  this->paths = paths;
}

/** initialize() is meant to read and return summary information to the foxglove
 * application about the set of files being read. The loader should also read any index information
 * that it needs to iterate over messages in initialize(). For simplicity, this loader reads entire
 * input files and indexes their line endings, but more sophisticated formats should not need to
 * be read from front to back.
 */
Result<Initialization> LCMDataLoader::initialize()
{
  if (paths.size() != 1)
  {
    return Result<Initialization>{.error = "Only one file supported"};
  }
  const std::string &path = paths[0];

  std::vector<Schema> schemas = {
      Schema{
          .id = SCHEMA_POSES_IN_FRAME,
          .name = "foxglove.PosesInFrame",
          .encoding = "protobuf",
          .data = BytesView{
              .ptr = PosesInFrame_bin,
              .len = PosesInFrame_bin_len,
          },
      },
      Schema{
          .id = SCHEMA_LOCATION_FIX,
          .name = "foxglove.LocationFix",
          .encoding = "protobuf",
          .data = BytesView{
              .ptr = LocationFix_bin,
              .len = LocationFix_bin_len,
          },
      },
      Schema{
          .id = SCHEMA_POINT_CLOUD,
          .name = "foxglove.PointCloud",
          .encoding = "protobuf",
          .data = BytesView{
              .ptr = PointCloud_bin,
              .len = PointCloud_bin_len,
          },
      },
      Schema{
          .id = SCHEMA_LASER_SCAN,
          .name = "foxglove.LaserScan",
          .encoding = "protobuf",
          .data = BytesView{
              .ptr = LaserScan_bin,
              .len = LaserScan_bin_len,
          },
      },
      Schema{
          .id = SCHEMA_COMPRESSED_IMAGE,
          .name = "foxglove.CompressedImage",
          .encoding = "protobuf",
          .data = BytesView{
              .ptr = CompressedImage_bin,
              .len = CompressedImage_bin_len,
          },
      },
  };
  std::vector<Channel> channels = {
      // Channel{
      //     .id = CHANNEL_POSE,
      //     .schema_id = SCHEMA_POSES_IN_FRAME,
      //     .topic_name = "POSE",
      //     .message_encoding = "protobuf",
      //     .message_count = 0,
      // },
      Channel{
          .id = CHANNEL_CAM_THUMB_RFR,
          .schema_id = SCHEMA_COMPRESSED_IMAGE,
          .topic_name = "CAM_THUMB_RFR",
          .message_encoding = "protobuf",
          .message_count = 0,
      },
      Channel{
          .id = CHANNEL_CAM_THUMB_RFC,
          .schema_id = SCHEMA_COMPRESSED_IMAGE,
          .topic_name = "CAM_THUMB_RFC",
          .message_encoding = "protobuf",
          .message_count = 0,
      },
      // Channel{
      //     .id = CHANNEL_GPS_TO_LOCAL,
      //     .schema_id = SCHEMA_LOCATION_FIX,
      //     .topic_name = "GPS_TO_LOCAL",
      //     .message_encoding = "protobuf",
      //     .message_count = 0,
      // },
      Channel{
          .id = CHANNEL_VELODYNE,
          .schema_id = SCHEMA_POINT_CLOUD,
          .topic_name = "VELODYNE",
          .message_encoding = "protobuf",
          .message_count = 0,
      },
      Channel{
          .id = CHANNEL_BROOM_L,
          .schema_id = SCHEMA_LASER_SCAN,
          .topic_name = "BROOM_L",
          .message_encoding = "protobuf",
          .message_count = 0,
      },
      Channel{
          .id = CHANNEL_BROOM_R,
          .schema_id = SCHEMA_LASER_SCAN,
          .topic_name = "BROOM_R",
          .message_encoding = "protobuf",
          .message_count = 0,
      },
      Channel{
          .id = CHANNEL_BROOM_C,
          .schema_id = SCHEMA_LASER_SCAN,
          .topic_name = "BROOM_C",
          .message_encoding = "protobuf",
          .message_count = 0,
      },
      Channel{
          .id = CHANNEL_BROOM_CL,
          .schema_id = SCHEMA_LASER_SCAN,
          .topic_name = "BROOM_CL",
          .message_encoding = "protobuf",
          .message_count = 0,
      },
      Channel{
          .id = CHANNEL_BROOM_CR,
          .schema_id = SCHEMA_LASER_SCAN,
          .topic_name = "BROOM_CR",
          .message_encoding = "protobuf",
          .message_count = 0,
      },
  };

  uint64_t start_time_ns = UINT64_MAX;
  uint64_t end_time_ns = 0;
  {
    Reader reader = Reader::open(path.c_str());
    buffer.resize(reader.size());
    uint64_t n_read = reader.read(buffer.data(), buffer.size());
    if (n_read != buffer.size())
    {
      return Result<Initialization>{.error = "failed to read entire file"};
    }
  }

  {
    LCMEvent event = {0};
    std::vector<uint8_t> scratch;
    const uint8_t *buf = buffer.data();
    size_t pos = 0;
    while (pos != buffer.size())
    {
      int64_t read = read_next(buf + pos, buffer.size() - pos, &event);
      if (read < 0)
      {
        if (read == UNEXPECTED_EOF)
        {
          return Result<Initialization>{.error = "failed to decode LCM log: unexpected EOF"};
        }
        else if (read == MALFORMED_EVENT)
        {
          return Result<Initialization>{.error = "failed to decode LCM log: malformed event"};
        }
        else
        {
          return Result<Initialization>{.error = "failed to decode LCM log: unknown error"};
        }
      }
      if (read == 0)
      {
        break;
      }
      for (Channel &channel : channels)
      {
        if (channel.topic_name == event.channel)
        {
          channel.message_count = *channel.message_count + 1;
          uint64_t timestamp_ns = event.timestamp_us * 1000;
          if (timestamp_ns < start_time_ns)
          {
            start_time_ns = timestamp_ns;
          }
          if (timestamp_ns > end_time_ns)
          {
            end_time_ns = timestamp_ns;
          }
          index.push_back(EventIndex{
              .offset = pos,
              .channel_id = channel.id,
              .schema_id = *channel.schema_id,
              .timestamp_ns = timestamp_ns,
          });
        }
      }
      pos += read;
    }
  }
  return Result<Initialization>{
      .value =
          Initialization{
              .channels = channels,
              .schemas = schemas,
              .time_range =
                  TimeRange{
                      .start_time = start_time_ns,
                      .end_time = end_time_ns,
                  }}};
}
/** returns an AbstractMessageIterator for the set of requested args.
 * More than one message iterator may be instantiated at a given time.
 */
Result<std::unique_ptr<AbstractMessageIterator>> LCMDataLoader::create_iterator(
    const MessageIteratorArgs &args)
{
  return Result<std::unique_ptr<AbstractMessageIterator>>{
      .value = std::make_unique<LCMMessageIterator>(this, args),
  };
}

LCMMessageIterator::LCMMessageIterator(LCMDataLoader *loader, MessageIteratorArgs args_) : data_loader(loader), args(args_)
{
  for (index_pos = 0; index_pos < data_loader->index.size(); index_pos++)
  {
    const EventIndex &index = data_loader->index[index_pos];
    if (index.timestamp_ns < args.start_time)
    {
      continue;
    }
    if (index.timestamp_ns > args.end_time)
    {
      // did not find any relevant messages, set position to EOF
      index_pos = data_loader->index.size();
      return;
    }
    for (uint16_t channel_id : args.channel_ids)
    {
      if (index.channel_id == channel_id)
      {
        return;
      }
    }
  }
}

/** `next()` returns the next message from the loaded files that matches the arguments provided to
 * `create_iterator(args)`. If none are left to read, it returns std::nullopt.
 */
std::optional<Result<Message>> LCMMessageIterator::next()
{
  for (; index_pos < data_loader->index.size(); index_pos++)
  {
    EventIndex index = data_loader->index[index_pos];
    if (index.timestamp_ns > args.end_time)
    {
      return std::nullopt;
    }
    for (uint16_t channel_id : args.channel_ids)
    {
      if (index.channel_id == channel_id)
      {
        const uint8_t *start = data_loader->buffer.data() + index.offset;
        const size_t remaining = data_loader->buffer.size() - index.offset;
        if (read_next(start, remaining, &current_event) < 0)
        {
          error("failed to parse event at offset", index.offset);
          return Result<Message>{.error = "failed to parse event"};
        }
        if (index.channel_id == CHANNEL_POSE)
        {
          transcoder.transcode_poses_in_frame(current_event.data, &last_serialized_message, "poses");
        }
        else if (index.channel_id == CHANNEL_BROOM_C)
        {
          transcoder.transcode_laser_scan(current_event.data, &last_serialized_message, "broom_c");
        }
        else if (index.channel_id == CHANNEL_BROOM_L)
        {
          transcoder.transcode_laser_scan(current_event.data, &last_serialized_message, "broom_l");
        }
        else if (index.channel_id == CHANNEL_BROOM_R)
        {
          transcoder.transcode_laser_scan(current_event.data, &last_serialized_message, "broom_r");
        }
        else if (index.channel_id == CHANNEL_BROOM_CL)
        {
          transcoder.transcode_laser_scan(current_event.data, &last_serialized_message, "broom_cl");
        }
        else if (index.channel_id == CHANNEL_BROOM_CR)
        {
          transcoder.transcode_laser_scan(current_event.data, &last_serialized_message, "broom_cr");
        }
        else if (index.channel_id == CHANNEL_CAM_THUMB_RFC)
        {
          transcoder.transcode_image(current_event.data, &last_serialized_message, "cam_thumb_rfc");
        }
        else if (index.channel_id == CHANNEL_CAM_THUMB_RFR)
        {
          transcoder.transcode_image(current_event.data, &last_serialized_message, "cam_thumb_rfr");
        }
        else if (index.channel_id == CHANNEL_GPS_TO_LOCAL)
        {
          transcoder.transcode_gps(current_event.data, &last_serialized_message, "gps");
        }
        else if (index.channel_id == CHANNEL_VELODYNE)
        {
          transcoder.transcode_point_cloud(current_event.data, &last_serialized_message, "velodyne");
        }
        else
        {
          error("unrecognized indexed channel", index.channel_id);
          return Result<Message>{.error = "unrecognized indexed channel"};
        }
        index_pos++;
        return Result<Message>{
            .value = Message{
                .channel_id = channel_id,
                .log_time = index.timestamp_ns,
                .publish_time = index.timestamp_ns,
                .data = BytesView{
                    .ptr = last_serialized_message.data(),
                    .len = last_serialized_message.size(),
                }}};
      }
    }
  }
  return std::nullopt;
}

/** `construct_data_loader` is the hook you implement to load your data loader implementation. */
std::unique_ptr<AbstractDataLoader> construct_data_loader(const DataLoaderArgs &args)
{
  return std::make_unique<LCMDataLoader>(args.paths);
}
