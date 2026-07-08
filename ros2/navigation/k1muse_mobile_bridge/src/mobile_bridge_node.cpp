#include "k1muse_mobile_bridge/protocol.hpp"
#include "k1muse_mobile_bridge/rfcomm_transport.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <zlib.h>

#include <algorithm>
#include <atomic>
#include <array>
#include <cstdlib>
#include <cmath>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <iterator>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <sys/wait.h>

namespace k1muse_mobile_bridge
{
namespace
{
using namespace std::chrono_literals;

constexpr const char * kDebugLogDir =
  "/home/bianbu/k1muse_communicate_ros/src/k1muse_mobile_bridge/debug_logs";

int64_t wall_ms()
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

void append_debug_line(const std::string & file_name, const std::string & event, const std::string & fields)
{
  static std::mutex log_mutex;
  std::lock_guard<std::mutex> lock(log_mutex);
  std::filesystem::create_directories(kDebugLogDir);
  std::ofstream out(std::filesystem::path(kDebugLogDir) / file_name, std::ios::app);
  out << wall_ms() << '\t' << event << '\t' << fields << '\n';
}

enum class FramePriority : uint8_t
{
  HIGH = 0,
  NORMAL = 1,
  LOW = 2,
};

struct QueuedFrame
{
  MessageType type;
  FramePriority priority;
  std::vector<uint8_t> frame;
};

const char * message_type_name(MessageType type)
{
  switch (type) {
    case MessageType::HELLO: return "HELLO";
    case MessageType::HEARTBEAT: return "HEARTBEAT";
    case MessageType::MAP_INFO: return "MAP_INFO";
    case MessageType::MAP_TILE: return "MAP_TILE";
    case MessageType::ROBOT_POSE: return "ROBOT_POSE";
    case MessageType::TELEOP_CMD: return "TELEOP_CMD";
    case MessageType::STOP: return "STOP";
    case MessageType::NAV_PATH: return "NAV_PATH";
    case MessageType::BRIDGE_CONTROL: return "BRIDGE_CONTROL";
    case MessageType::BRIDGE_STATUS: return "BRIDGE_STATUS";
    case MessageType::MAP_CONTROL: return "MAP_CONTROL";
    case MessageType::MAP_CONTROL_STATUS: return "MAP_CONTROL_STATUS";
    case MessageType::MAP_LIBRARY_REQUEST: return "MAP_LIBRARY_REQUEST";
    case MessageType::MAP_LIBRARY_STATUS: return "MAP_LIBRARY_STATUS";
    case MessageType::MAP_LIBRARY_LIST: return "MAP_LIBRARY_LIST";
    case MessageType::MAP_REGIONS_DATA: return "MAP_REGIONS_DATA";
    case MessageType::ERROR: return "ERROR";
    default: return "OTHER";
  }
}

enum class BridgeCommand : uint8_t
{
  START_BRIDGE = 1,
  STOP_BRIDGE = 2,
  QUERY_BRIDGE = 3,
};

enum class BridgeState : uint8_t
{
  SUPERVISOR = 0,
  STARTING = 1,
  ONLINE = 2,
  STOPPING = 3,
  ERROR = 4,
};

enum class MapCommand : uint8_t
{
  START_MAPPING = 1,
  SAVE_MAP_MANUAL = 2,
  STOP_MAPPING = 3,
  QUERY_MAPPING = 4,
};

enum class MapState : uint8_t
{
  IDLE = 0,
  STARTING = 1,
  MAPPING = 2,
  SAVING = 3,
  STOPPING = 4,
  ERROR = 5,
};

enum class MappingMode : uint8_t
{
  MANUAL = 0,
  AUTO = 1,
};

enum class RoomSize : uint8_t
{
  SMALL = 0,
  MEDIUM = 1,
  LARGE = 2,
  CUSTOM = 3,
};

enum class MapLibraryCommand : uint8_t
{
  LIST_MAPS = 1,
  LOAD_MAP = 2,
  READ_REGIONS = 3,
  SAVE_REGIONS = 4,
};

struct StaticMap
{
  std::string yaml_name;
  std::string image_name;
  uint32_t width = 0;
  uint32_t height = 0;
  float resolution = 0.0F;
  float origin_x = 0.0F;
  float origin_y = 0.0F;
  float origin_yaw = 0.0F;
  std::vector<int8_t> data;
};

std::vector<uint8_t> zlib_compress(const std::vector<uint8_t> & raw)
{
  uLongf compressed_len = compressBound(static_cast<uLong>(raw.size()));
  std::vector<uint8_t> compressed(compressed_len);

  int rc = compress2(
    compressed.data(), &compressed_len,
    raw.data(), static_cast<uLong>(raw.size()),
    Z_BEST_SPEED);
  if (rc != Z_OK) {
    return raw;
  }

  compressed.resize(static_cast<size_t>(compressed_len));
  return compressed;
}

uint32_t payload_crc(const std::vector<uint8_t> & data)
{
  return static_cast<uint32_t>(crc32(0L, data.data(), static_cast<uInt>(data.size())));
}

uint16_t read_u16_le(const std::vector<uint8_t> & data, size_t offset)
{
  return static_cast<uint16_t>(data[offset]) |
    (static_cast<uint16_t>(data[offset + 1]) << 8U);
}

uint32_t read_u32_le(const std::vector<uint8_t> & data, size_t offset)
{
  return static_cast<uint32_t>(data[offset]) |
    (static_cast<uint32_t>(data[offset + 1]) << 8U) |
    (static_cast<uint32_t>(data[offset + 2]) << 16U) |
    (static_cast<uint32_t>(data[offset + 3]) << 24U);
}

float read_float_le(const std::vector<uint8_t> & data, size_t offset)
{
  const uint32_t raw = read_u32_le(data, offset);
  float value = 0.0F;
  static_assert(sizeof(raw) == sizeof(value), "float must be 32-bit");
  std::memcpy(&value, &raw, sizeof(value));
  return value;
}

std::string read_string_le(const std::vector<uint8_t> & data, size_t * offset)
{
  if (*offset + 2U > data.size()) {
    return "";
  }
  const uint16_t len = read_u16_le(data, *offset);
  *offset += 2U;
  if (*offset + len > data.size()) {
    *offset = data.size();
    return "";
  }
  std::string value(data.begin() + static_cast<std::ptrdiff_t>(*offset),
    data.begin() + static_cast<std::ptrdiff_t>(*offset + len));
  *offset += len;
  return value;
}

uint32_t stamp_ms(const rclcpp::Time & time)
{
  return static_cast<uint32_t>(time.nanoseconds() / 1000000ULL);
}

std::string shell_quote(const std::string & value)
{
  std::string out = "'";
  for (const char ch : value) {
    if (ch == '\'') {
      out += "'\\''";
    } else {
      out.push_back(ch);
    }
  }
  out.push_back('\'');
  return out;
}

int run_command(const std::string & command, std::string * output)
{
  std::array<char, 256> buffer{};
  FILE * pipe = popen((command + " 2>&1").c_str(), "r");
  if (pipe == nullptr) {
    if (output != nullptr) {
      *output = "failed to start command";
    }
    return 127;
  }

  std::string collected;
  while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) != nullptr) {
    collected += buffer.data();
    if (collected.size() > 4096U) {
      collected.erase(0U, collected.size() - 4096U);
    }
  }

  const int rc = pclose(pipe);
  if (output != nullptr) {
    *output = collected;
  }
  if (rc == -1) {
    return 127;
  }
  if (WIFEXITED(rc)) {
    return WEXITSTATUS(rc);
  }
  return 128;
}

std::string last_non_empty_line(const std::string & text)
{
  size_t end = text.find_last_not_of("\r\n\t ");
  if (end == std::string::npos) {
    return "";
  }
  const size_t begin = text.find_last_of("\r\n", end);
  return text.substr(begin == std::string::npos ? 0U : begin + 1U, end - (begin == std::string::npos ? 0U : begin + 1U) + 1U);
}

std::string extract_marker(const std::string & text, const std::string & marker)
{
  const size_t pos = text.find(marker);
  if (pos == std::string::npos) {
    return "";
  }
  const size_t value_start = pos + marker.size();
  const size_t value_end = text.find_first_of("\r\n", value_start);
  return text.substr(value_start, value_end == std::string::npos ? std::string::npos : value_end - value_start);
}

bool contains_line(const std::string & text, const std::string & needle)
{
  std::istringstream stream(text);
  std::string line;
  while (std::getline(stream, line)) {
    if (line == needle) {
      return true;
    }
  }
  return false;
}

bool contains_any(const std::string & text, const std::vector<std::string> & needles)
{
  for (const auto & needle : needles) {
    if (text.find(needle) != std::string::npos) {
      return true;
    }
  }
  return false;
}

std::string trim_copy(const std::string & value)
{
  const size_t begin = value.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const size_t end = value.find_last_not_of(" \t\r\n");
  return value.substr(begin, end - begin + 1U);
}

std::string unquote_copy(const std::string & value)
{
  const std::string trimmed = trim_copy(value);
  if (trimmed.size() >= 2U &&
    ((trimmed.front() == '"' && trimmed.back() == '"') ||
    (trimmed.front() == '\'' && trimmed.back() == '\'')))
  {
    return trimmed.substr(1U, trimmed.size() - 2U);
  }
  return trimmed;
}

bool safe_map_yaml_name(const std::string & name)
{
  if (name.empty() || name.size() > 160U) {
    return false;
  }
  if (name.find('/') != std::string::npos || name.find('\\') != std::string::npos) {
    return false;
  }
  if (name == "." || name == "..") {
    return false;
  }
  const std::filesystem::path path(name);
  return path.extension() == ".yaml" || path.extension() == ".yml";
}

std::optional<float> parse_float_after_colon(const std::string & line)
{
  const size_t colon = line.find(':');
  if (colon == std::string::npos) {
    return std::nullopt;
  }
  try {
    return std::stof(trim_copy(line.substr(colon + 1U)));
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

std::optional<std::array<float, 3>> parse_origin_after_colon(const std::string & line)
{
  const size_t left = line.find('[');
  const size_t right = line.find(']', left == std::string::npos ? 0U : left);
  if (left == std::string::npos || right == std::string::npos || right <= left) {
    return std::nullopt;
  }
  std::array<float, 3> values{0.0F, 0.0F, 0.0F};
  std::istringstream stream(line.substr(left + 1U, right - left - 1U));
  std::string item;
  for (size_t i = 0; i < values.size(); ++i) {
    if (!std::getline(stream, item, ',')) {
      return std::nullopt;
    }
    try {
      values[i] = std::stof(trim_copy(item));
    } catch (const std::exception &) {
      return std::nullopt;
    }
  }
  return values;
}

template<typename QuaternionT>
float yaw_from_quaternion(const QuaternionT & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return static_cast<float>(std::atan2(siny_cosp, cosy_cosp));
}
}  // namespace

class MobileBridgeNode : public rclcpp::Node
{
public:
  MobileBridgeNode()
  : Node("mobile_bridge_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    rfcomm_device_ = declare_parameter<std::string>("rfcomm_device", "/dev/rfcomm0");
    map_topic_ = declare_parameter<std::string>("map_topic", "/map");
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    mapping_scripts_dir_ = declare_parameter<std::string>(
      "mapping_scripts_dir",
      "/home/bianbu/k1muse_slam_ros/install/k1muse_slam_nav/lib/k1muse_slam_nav");
    exploration_scripts_dir_ = declare_parameter<std::string>(
      "exploration_scripts_dir",
      "/home/bianbu/k1muse_slam_ros/install/k1muse_exploration/lib/k1muse_exploration");
    exploration_path_topic_ = declare_parameter<std::string>("exploration_path_topic", "/plan");
    map_library_dir_ = declare_parameter<std::string>(
      "map_library_dir",
      "/home/bianbu/k1muse_slam_ros/src/k1muse_slam_nav/maps");
    tile_size_ = declare_parameter<int>("tile_size", 64);
    max_queue_frames_ = declare_parameter<int>("max_queue_frames", 256);
    max_low_queue_frames_ = declare_parameter<int>("max_low_queue_frames", 8);
    max_map_tiles_per_update_ = declare_parameter<int>("max_map_tiles_per_update", 8);
    map_publish_hz_ = declare_parameter<double>("map_publish_hz", 1.0);
    pose_publish_hz_ = declare_parameter<double>("pose_publish_hz", 10.0);
    path_publish_hz_ = declare_parameter<double>("path_publish_hz", 1.0);
    teleop_timeout_ms_ = declare_parameter<int>("teleop_timeout_ms", 300);

    tile_size_ = std::clamp(tile_size_, 16, 256);
    max_queue_frames_ = std::max(max_queue_frames_, 16);
    max_low_queue_frames_ = std::clamp(max_low_queue_frames_, 1, max_queue_frames_);
    max_map_tiles_per_update_ = std::clamp(max_map_tiles_per_update_, 1, 128);
    map_publish_hz_ = std::clamp(map_publish_hz_, 0.1, 5.0);
    pose_publish_hz_ = std::clamp(pose_publish_hz_, 1.0, 30.0);
    path_publish_hz_ = std::clamp(path_publish_hz_, 0.2, 5.0);
    teleop_timeout_ms_ = std::clamp(teleop_timeout_ms_, 100, 2000);

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&MobileBridgeNode::on_map, this, std::placeholders::_1));
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      exploration_path_topic_, rclcpp::QoS(1).best_effort(),
      std::bind(&MobileBridgeNode::on_path, this, std::placeholders::_1));

    connect_timer_ = create_wall_timer(1s, std::bind(&MobileBridgeNode::try_connect, this));
    pose_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / pose_publish_hz_),
      std::bind(&MobileBridgeNode::send_pose, this));
    heartbeat_timer_ = create_wall_timer(1s, std::bind(&MobileBridgeNode::send_heartbeat, this));
    teleop_timer_ = create_wall_timer(100ms, std::bind(&MobileBridgeNode::check_teleop_timeout, this));

    tx_thread_ = std::thread(&MobileBridgeNode::tx_loop, this);
    rx_thread_ = std::thread(&MobileBridgeNode::rx_loop, this);

    RCLCPP_INFO(
      get_logger(),
      "Mobile bridge ready: rfcomm=%s map=%s path=%s cmd_vel=%s frames=%s->%s tile=%d map_hz=%.2f pose_hz=%.2f path_hz=%.2f",
      rfcomm_device_.c_str(), map_topic_.c_str(), exploration_path_topic_.c_str(), cmd_vel_topic_.c_str(),
      global_frame_.c_str(), base_frame_.c_str(), tile_size_, map_publish_hz_, pose_publish_hz_,
      path_publish_hz_);
  }

  ~MobileBridgeNode() override
  {
    cleanup_robot_processes("mobile bridge shutdown");
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      stopping_ = true;
    }
    queue_cv_.notify_all();
    if (tx_thread_.joinable()) {
      tx_thread_.join();
    }
    if (rx_thread_.joinable()) {
      rx_thread_.join();
    }
    transport_.close_device();
    for (auto & worker : worker_threads_) {
      if (worker.joinable()) {
        worker.join();
      }
    }
  }

private:
  void try_connect()
  {
    if (transport_.is_open()) {
      return;
    }

    if (!rfcomm_link_connected()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for active mobile RFCOMM link on %s", rfcomm_device_.c_str());
      return;
    }

    if (!transport_.open_device(rfcomm_device_)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for mobile RFCOMM device %s", rfcomm_device_.c_str());
      return;
    }

    map_info_sent_ = false;
    tile_hash_.clear();
    rx_buffer_.clear();
    RCLCPP_INFO(get_logger(), "Mobile RFCOMM connected on %s", rfcomm_device_.c_str());
    send_hello();
  }

  bool rfcomm_link_connected() const
  {
    const auto slash = rfcomm_device_.find_last_of('/');
    const std::string device_name =
      slash == std::string::npos ? rfcomm_device_ : rfcomm_device_.substr(slash + 1U);

    std::string output;
    if (run_command("rfcomm 2>/dev/null", &output) != 0 && output.empty()) {
      return false;
    }

    std::istringstream stream(output);
    std::string line;
    const std::string prefix = device_name + ":";
    while (std::getline(stream, line)) {
      if (line.rfind(prefix, 0U) == 0U) {
        return line.find(" closed ") == std::string::npos;
      }
    }
    return false;
  }

  void send_hello()
  {
    std::vector<uint8_t> payload;
    append_string(payload, "k1muse_mobile_bridge");
    append_u8(payload, 1);
    enqueue(MessageType::HELLO, payload, FramePriority::HIGH);
  }

  void send_heartbeat()
  {
    std::vector<uint8_t> payload;
    append_u32(payload, static_cast<uint32_t>(now().nanoseconds() / 1000000ULL));
    enqueue(MessageType::HEARTBEAT, payload, FramePriority::HIGH);
  }

  void on_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    const auto now_time = now();
    if (last_map_tx_.nanoseconds() != 0 &&
      (now_time - last_map_tx_).seconds() < (1.0 / map_publish_hz_))
    {
      return;
    }
    last_map_tx_ = now_time;

    const auto & info = msg->info;
    if (info.width == 0U || info.height == 0U || msg->data.empty()) {
      return;
    }

    const float next_origin_x = static_cast<float>(info.origin.position.x);
    const float next_origin_y = static_cast<float>(info.origin.position.y);
    const float next_origin_yaw = yaw_from_quaternion(info.origin.orientation);
    const float next_resolution = info.resolution;
    const float origin_tolerance = std::max(0.001F, next_resolution * 0.5F);
    const bool metadata_changed =
      !map_info_sent_ ||
      info.width != map_width_ ||
      info.height != map_height_ ||
      std::abs(next_resolution - map_resolution_) > 0.000001F ||
      std::abs(next_origin_x - map_origin_x_) > origin_tolerance ||
      std::abs(next_origin_y - map_origin_y_) > origin_tolerance ||
      std::abs(next_origin_yaw - map_origin_yaw_) > 0.01F;

    if (metadata_changed) {
      map_version_++;
      map_width_ = info.width;
      map_height_ = info.height;
      map_resolution_ = next_resolution;
      map_origin_x_ = next_origin_x;
      map_origin_y_ = next_origin_y;
      map_origin_yaw_ = next_origin_yaw;
      const size_t tile_count = tile_count_for_map(map_width_, map_height_);
      tile_hash_.assign(tile_count, 0U);
      send_map_info();
      map_info_sent_ = true;
    }

    append_debug_line(
      "mobile_bridge_debug.tsv", "map_update",
      "w=" + std::to_string(info.width) +
      "\th=" + std::to_string(info.height) +
      "\tversion=" + std::to_string(map_version_) +
      "\tmetadata_changed=" + std::to_string(metadata_changed ? 1 : 0) +
      "\torigin_dx=" + std::to_string(next_origin_x - map_origin_x_) +
      "\torigin_dy=" + std::to_string(next_origin_y - map_origin_y_));
    send_changed_tiles(*msg);
  }

  size_t tile_count_for_map(uint32_t width, uint32_t height) const
  {
    const uint32_t tiles_x = (width + static_cast<uint32_t>(tile_size_) - 1U) /
      static_cast<uint32_t>(tile_size_);
    const uint32_t tiles_y = (height + static_cast<uint32_t>(tile_size_) - 1U) /
      static_cast<uint32_t>(tile_size_);
    return static_cast<size_t>(tiles_x) * static_cast<size_t>(tiles_y);
  }

  void send_map_info()
  {
    std::vector<uint8_t> payload;
    append_u32(payload, map_version_);
    append_u32(payload, map_width_);
    append_u32(payload, map_height_);
    append_float(payload, map_resolution_);
    append_float(payload, map_origin_x_);
    append_float(payload, map_origin_y_);
    append_float(payload, map_origin_yaw_);
    append_u16(payload, static_cast<uint16_t>(tile_size_));
    enqueue(MessageType::MAP_INFO, payload, FramePriority::NORMAL);
  }

  void send_changed_tiles(const nav_msgs::msg::OccupancyGrid & msg)
  {
    int sent_tiles = 0;
    int same_tiles = 0;
    int dropped_tiles = 0;
    bool stopped_by_teleop = false;
    bool stopped_by_limit = false;
    const uint32_t tiles_x = (map_width_ + static_cast<uint32_t>(tile_size_) - 1U) /
      static_cast<uint32_t>(tile_size_);
    const uint32_t tiles_y = (map_height_ + static_cast<uint32_t>(tile_size_) - 1U) /
      static_cast<uint32_t>(tile_size_);

    for (uint32_t ty = 0; ty < tiles_y; ++ty) {
      for (uint32_t tx = 0; tx < tiles_x; ++tx) {
        if (sent_tiles >= max_map_tiles_per_update_) {
          stopped_by_limit = true;
          append_debug_line(
            "mobile_bridge_debug.tsv", "map_tiles",
            "sent=" + std::to_string(sent_tiles) +
            "\tsame=" + std::to_string(same_tiles) +
            "\tdropped=" + std::to_string(dropped_tiles) +
            "\tstop=limit");
          return;
        }
        if (recent_teleop_active()) {
          stopped_by_teleop = true;
          append_debug_line(
            "mobile_bridge_debug.tsv", "map_tiles",
            "sent=" + std::to_string(sent_tiles) +
            "\tsame=" + std::to_string(same_tiles) +
            "\tdropped=" + std::to_string(dropped_tiles) +
            "\tstop=teleop");
          return;
        }
        const uint32_t x0 = tx * static_cast<uint32_t>(tile_size_);
        const uint32_t y0 = ty * static_cast<uint32_t>(tile_size_);
        const uint16_t tile_w = static_cast<uint16_t>(
          std::min<uint32_t>(static_cast<uint32_t>(tile_size_), map_width_ - x0));
        const uint16_t tile_h = static_cast<uint16_t>(
          std::min<uint32_t>(static_cast<uint32_t>(tile_size_), map_height_ - y0));

        std::vector<uint8_t> raw;
        raw.reserve(static_cast<size_t>(tile_w) * static_cast<size_t>(tile_h));
        for (uint32_t row = 0; row < tile_h; ++row) {
          const size_t offset = static_cast<size_t>(y0 + row) * map_width_ + x0;
          for (uint32_t col = 0; col < tile_w; ++col) {
            raw.push_back(static_cast<uint8_t>(msg.data[offset + col]));
          }
        }

        const size_t index = static_cast<size_t>(ty) * tiles_x + tx;
        const uint32_t hash = payload_crc(raw);
        if (index < tile_hash_.size() && tile_hash_[index] == hash) {
          ++same_tiles;
          continue;
        }
        const std::vector<uint8_t> compressed = zlib_compress(raw);
        const bool use_zlib = compressed.size() < raw.size();
        const auto & bytes = use_zlib ? compressed : raw;

        std::vector<uint8_t> payload;
        append_u32(payload, map_version_);
        append_u32(payload, x0);
        append_u32(payload, y0);
        append_u16(payload, tile_w);
        append_u16(payload, tile_h);
        append_u32(payload, static_cast<uint32_t>(raw.size()));
        append_u8(payload, static_cast<uint8_t>(use_zlib ? TileEncoding::ZLIB : TileEncoding::RAW));
        payload.insert(payload.end(), bytes.begin(), bytes.end());

        if (enqueue(MessageType::MAP_TILE, payload, FramePriority::LOW) &&
          index < tile_hash_.size())
        {
          tile_hash_[index] = hash;
          ++sent_tiles;
        } else {
          ++dropped_tiles;
        }
      }
    }
    (void)stopped_by_teleop;
    (void)stopped_by_limit;
    append_debug_line(
      "mobile_bridge_debug.tsv", "map_tiles",
      "sent=" + std::to_string(sent_tiles) +
      "\tsame=" + std::to_string(same_tiles) +
      "\tdropped=" + std::to_string(dropped_tiles) +
      "\tstop=end");
  }

  std::filesystem::path map_library_path() const
  {
    return std::filesystem::path(map_library_dir_);
  }

  std::filesystem::path yaml_path_for_name(const std::string & yaml_name) const
  {
    return map_library_path() / yaml_name;
  }

  std::filesystem::path regions_path_for_name(const std::string & yaml_name) const
  {
    std::filesystem::path path = yaml_path_for_name(yaml_name);
    path.replace_extension(".regions.json");
    return path;
  }

  std::optional<StaticMap> read_static_map(const std::string & yaml_name, std::string * error) const
  {
    if (!safe_map_yaml_name(yaml_name)) {
      if (error != nullptr) {
        *error = "bad map name";
      }
      return std::nullopt;
    }

    const auto yaml_path = yaml_path_for_name(yaml_name);
    std::ifstream yaml(yaml_path);
    if (!yaml) {
      if (error != nullptr) {
        *error = "map yaml not found";
      }
      return std::nullopt;
    }

    StaticMap map;
    map.yaml_name = yaml_name;
    std::string image_name;
    std::string line;
    while (std::getline(yaml, line)) {
      const std::string trimmed = trim_copy(line);
      if (trimmed.rfind("image:", 0U) == 0U) {
        image_name = unquote_copy(trimmed.substr(trimmed.find(':') + 1U));
      } else if (trimmed.rfind("resolution:", 0U) == 0U) {
        auto value = parse_float_after_colon(trimmed);
        if (value.has_value()) {
          map.resolution = value.value();
        }
      } else if (trimmed.rfind("origin:", 0U) == 0U) {
        auto origin = parse_origin_after_colon(trimmed);
        if (origin.has_value()) {
          map.origin_x = origin.value()[0];
          map.origin_y = origin.value()[1];
          map.origin_yaw = origin.value()[2];
        }
      }
    }
    if (image_name.empty() || map.resolution <= 0.0F ||
      image_name.find('/') != std::string::npos || image_name.find('\\') != std::string::npos)
    {
      if (error != nullptr) {
        *error = "invalid map yaml";
      }
      return std::nullopt;
    }
    map.image_name = image_name;

    const auto image_path = yaml_path.parent_path() / image_name;
    if (!read_pgm_as_occupancy(image_path, &map, error)) {
      return std::nullopt;
    }
    return map;
  }

  bool read_pgm_as_occupancy(
    const std::filesystem::path & path,
    StaticMap * map,
    std::string * error) const
  {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
      if (error != nullptr) {
        *error = "map image not found";
      }
      return false;
    }
    const std::string magic = next_pgm_token(input);
    const std::string width_token = next_pgm_token(input);
    const std::string height_token = next_pgm_token(input);
    const std::string max_token = next_pgm_token(input);
    if (magic != "P5" || width_token.empty() || height_token.empty() || max_token.empty()) {
      if (error != nullptr) {
        *error = "invalid pgm header";
      }
      return false;
    }

    int width = 0;
    int height = 0;
    int max_value = 0;
    try {
      width = std::stoi(width_token);
      height = std::stoi(height_token);
      max_value = std::stoi(max_token);
    } catch (const std::exception &) {
      if (error != nullptr) {
        *error = "invalid pgm dimensions";
      }
      return false;
    }
    if (width <= 0 || height <= 0 || max_value <= 0 || max_value > 255) {
      if (error != nullptr) {
        *error = "unsupported pgm dimensions";
      }
      return false;
    }

    input.get();
    std::vector<uint8_t> pixels(static_cast<size_t>(width) * static_cast<size_t>(height));
    input.read(reinterpret_cast<char *>(pixels.data()), static_cast<std::streamsize>(pixels.size()));
    if (input.gcount() != static_cast<std::streamsize>(pixels.size())) {
      if (error != nullptr) {
        *error = "truncated pgm";
      }
      return false;
    }

    map->width = static_cast<uint32_t>(width);
    map->height = static_cast<uint32_t>(height);
    map->data.resize(pixels.size());
    for (int row = 0; row < height; ++row) {
      for (int col = 0; col < width; ++col) {
        const size_t source_index = static_cast<size_t>(row) * static_cast<size_t>(width) +
          static_cast<size_t>(col);
        const size_t dest_index = static_cast<size_t>(height - 1 - row) * static_cast<size_t>(width) +
          static_cast<size_t>(col);
        const uint8_t pixel = pixels[source_index];
        if (pixel >= 250U) {
          map->data[dest_index] = 0;
        } else if (pixel <= 10U) {
          map->data[dest_index] = 100;
        } else {
          map->data[dest_index] = -1;
        }
      }
    }
    return true;
  }

  std::string next_pgm_token(std::ifstream & input) const
  {
    std::string token;
    while (input >> token) {
      if (!token.empty() && token[0] == '#') {
        std::string discard;
        std::getline(input, discard);
        continue;
      }
      return token;
    }
    return "";
  }

  void send_static_map(const StaticMap & map)
  {
    nav_msgs::msg::OccupancyGrid msg;
    msg.info.width = map.width;
    msg.info.height = map.height;
    msg.info.resolution = map.resolution;
    msg.info.origin.position.x = map.origin_x;
    msg.info.origin.position.y = map.origin_y;
    msg.info.origin.orientation.w = std::cos(map.origin_yaw / 2.0F);
    msg.info.origin.orientation.z = std::sin(map.origin_yaw / 2.0F);
    msg.data = map.data;

    map_version_++;
    map_width_ = map.width;
    map_height_ = map.height;
    map_resolution_ = map.resolution;
    map_origin_x_ = map.origin_x;
    map_origin_y_ = map.origin_y;
    map_origin_yaw_ = map.origin_yaw;
    tile_hash_.assign(tile_count_for_map(map_width_, map_height_), 0U);
    send_map_info();
    map_info_sent_ = true;
    send_changed_tiles(msg);
  }

  void send_map_library_status(
    MapLibraryCommand command,
    bool success,
    const std::string & message)
  {
    std::vector<uint8_t> payload;
    append_u32(payload, stamp_ms(now()));
    append_u8(payload, static_cast<uint8_t>(command));
    append_u8(payload, success ? 1U : 0U);
    append_string(payload, message);
    enqueue(MessageType::MAP_LIBRARY_STATUS, payload, FramePriority::HIGH);
  }

  void send_map_library_list()
  {
    std::vector<std::string> yaml_names;
    const auto root = map_library_path();
    if (std::filesystem::exists(root)) {
      for (const auto & item : std::filesystem::directory_iterator(root)) {
        if (!item.is_regular_file()) {
          continue;
        }
        const auto path = item.path();
        if (path.extension() == ".yaml" || path.extension() == ".yml") {
          yaml_names.push_back(path.filename().string());
        }
      }
    }
    std::sort(yaml_names.begin(), yaml_names.end());

    std::vector<uint8_t> payload;
    append_u32(payload, stamp_ms(now()));
    append_u16(payload, static_cast<uint16_t>(std::min<size_t>(yaml_names.size(), 65535U)));
    for (const auto & yaml_name : yaml_names) {
      if (!safe_map_yaml_name(yaml_name)) {
        continue;
      }
      std::string image_name;
      std::ifstream yaml(yaml_path_for_name(yaml_name));
      std::string line;
      while (std::getline(yaml, line)) {
        const std::string trimmed = trim_copy(line);
        if (trimmed.rfind("image:", 0U) == 0U) {
          image_name = unquote_copy(trimmed.substr(trimmed.find(':') + 1U));
          break;
        }
      }
      append_string(payload, yaml_name);
      append_string(payload, image_name);
      append_u8(payload, std::filesystem::exists(regions_path_for_name(yaml_name)) ? 1U : 0U);
    }
    enqueue(MessageType::MAP_LIBRARY_LIST, payload, FramePriority::HIGH);
  }

  void send_regions_data(const std::string & yaml_name)
  {
    std::vector<uint8_t> payload;
    append_u32(payload, stamp_ms(now()));
    append_string(payload, yaml_name);

    std::string json;
    const auto path = regions_path_for_name(yaml_name);
    if (safe_map_yaml_name(yaml_name) && std::filesystem::exists(path)) {
      std::ifstream input(path);
      std::ostringstream stream;
      stream << input.rdbuf();
      json = stream.str();
      append_u8(payload, 1U);
    } else {
      append_u8(payload, 0U);
    }
    append_string(payload, json);
    enqueue(MessageType::MAP_REGIONS_DATA, payload, FramePriority::HIGH);
  }

  void handle_map_library_request(const std::vector<uint8_t> & payload)
  {
    if (payload.size() < 5U) {
      send_map_library_status(MapLibraryCommand::LIST_MAPS, false, "bad map library request");
      return;
    }
    const auto command = static_cast<MapLibraryCommand>(payload[4]);
    size_t offset = 5U;
    const std::string yaml_name = read_string_le(payload, &offset);
    const std::string json = read_string_le(payload, &offset);

    switch (command) {
      case MapLibraryCommand::LIST_MAPS:
        send_map_library_list();
        send_map_library_status(command, true, "map list sent");
        break;
      case MapLibraryCommand::LOAD_MAP:
        load_static_map_request(yaml_name);
        break;
      case MapLibraryCommand::READ_REGIONS:
        if (!safe_map_yaml_name(yaml_name)) {
          send_map_library_status(command, false, "bad map name");
          return;
        }
        send_regions_data(yaml_name);
        send_map_library_status(command, true, "regions sent");
        break;
      case MapLibraryCommand::SAVE_REGIONS:
        save_regions_request(yaml_name, json);
        break;
      default:
        send_map_library_status(command, false, "unknown map library command");
        break;
    }
  }

  void load_static_map_request(const std::string & yaml_name)
  {
    std::string error;
    const auto map = read_static_map(yaml_name, &error);
    if (!map.has_value()) {
      send_map_library_status(MapLibraryCommand::LOAD_MAP, false, error);
      return;
    }
    send_static_map(map.value());
    send_map_library_status(MapLibraryCommand::LOAD_MAP, true, "static map loaded");
  }

  void save_regions_request(const std::string & yaml_name, const std::string & json)
  {
    if (!safe_map_yaml_name(yaml_name) || json.empty()) {
      send_map_library_status(MapLibraryCommand::SAVE_REGIONS, false, "bad regions payload");
      return;
    }
    if (!std::filesystem::exists(yaml_path_for_name(yaml_name))) {
      send_map_library_status(MapLibraryCommand::SAVE_REGIONS, false, "map yaml not found");
      return;
    }
    const auto target = regions_path_for_name(yaml_name);
    const auto tmp = target.string() + ".tmp";
    {
      std::ofstream output(tmp, std::ios::trunc);
      if (!output) {
        send_map_library_status(MapLibraryCommand::SAVE_REGIONS, false, "failed to open regions tmp");
        return;
      }
      output << json;
      output << "\n";
      if (!output) {
        send_map_library_status(MapLibraryCommand::SAVE_REGIONS, false, "failed to write regions tmp");
        return;
      }
    }
    std::error_code ec;
    std::filesystem::rename(tmp, target, ec);
    if (ec) {
      std::filesystem::remove(target, ec);
      ec.clear();
      std::filesystem::rename(tmp, target, ec);
    }
    if (ec) {
      send_map_library_status(MapLibraryCommand::SAVE_REGIONS, false, "failed to replace regions file");
      return;
    }
    send_map_library_status(MapLibraryCommand::SAVE_REGIONS, true, "regions saved");
  }

  void send_pose()
  {
    float x = 0.0F;
    float y = 0.0F;
    float yaw = 0.0F;
    try {
      const auto tf = tf_buffer_.lookupTransform(global_frame_, base_frame_, tf2::TimePointZero);
      x = static_cast<float>(tf.transform.translation.x);
      y = static_cast<float>(tf.transform.translation.y);
      yaw = yaw_from_quaternion(tf.transform.rotation);
    } catch (const tf2::TransformException & ex) {
      try {
        const auto map_to_odom = tf_buffer_.lookupTransform(global_frame_, "odom", tf2::TimePointZero);
        const auto odom_to_base = tf_buffer_.lookupTransform("odom", base_frame_, tf2::TimePointZero);
        const float yaw_map_odom = yaw_from_quaternion(map_to_odom.transform.rotation);
        const float yaw_odom_base = yaw_from_quaternion(odom_to_base.transform.rotation);
        const float c = std::cos(yaw_map_odom);
        const float s = std::sin(yaw_map_odom);
        const float ox = static_cast<float>(odom_to_base.transform.translation.x);
        const float oy = static_cast<float>(odom_to_base.transform.translation.y);
        x = static_cast<float>(map_to_odom.transform.translation.x) + c * ox - s * oy;
        y = static_cast<float>(map_to_odom.transform.translation.y) + s * ox + c * oy;
        yaw = yaw_map_odom + yaw_odom_base;
      } catch (const tf2::TransformException &) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Waiting for TF %s -> %s: %s", global_frame_.c_str(), base_frame_.c_str(), ex.what());
        return;
      }
    }

    std::vector<uint8_t> payload;
    append_u32(payload, static_cast<uint32_t>(now().nanoseconds() / 1000000ULL));
    append_float(payload, x);
    append_float(payload, y);
    append_float(payload, yaw);
    enqueue(MessageType::ROBOT_POSE, payload, FramePriority::NORMAL);
  }

  void on_path(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (!auto_mapping_running() || msg->poses.size() < 2U) {
      return;
    }
    const auto now_time = now();
    if (last_path_tx_.nanoseconds() != 0 &&
      (now_time - last_path_tx_).seconds() < (1.0 / path_publish_hz_))
    {
      return;
    }
    last_path_tx_ = now_time;

    constexpr size_t max_points = 80U;
    const size_t total = msg->poses.size();
    const size_t stride = std::max<size_t>(1U, (total + max_points - 1U) / max_points);

    std::vector<uint8_t> payload;
    append_u32(payload, stamp_ms(now_time));
    append_u32(payload, ++path_version_);
    const size_t count_pos = payload.size();
    append_u16(payload, 0U);

    uint16_t count = 0U;
    for (size_t i = 0; i < total && count < max_points; i += stride) {
      append_float(payload, static_cast<float>(msg->poses[i].pose.position.x));
      append_float(payload, static_cast<float>(msg->poses[i].pose.position.y));
      ++count;
    }
    if (count < 2U) {
      return;
    }
    payload[count_pos] = static_cast<uint8_t>(count & 0xffU);
    payload[count_pos + 1U] = static_cast<uint8_t>((count >> 8U) & 0xffU);
    enqueue(MessageType::NAV_PATH, payload, FramePriority::NORMAL);
  }

  void send_bridge_status(
    BridgeState state,
    BridgeCommand command,
    bool success,
    const std::string & message)
  {
    std::vector<uint8_t> payload;
    append_u32(payload, stamp_ms(now()));
    append_u8(payload, static_cast<uint8_t>(state));
    append_u8(payload, static_cast<uint8_t>(command));
    append_u8(payload, success ? 1U : 0U);
    append_string(payload, message);
    enqueue(MessageType::BRIDGE_STATUS, payload, FramePriority::HIGH);
  }

  void send_map_status(
    MapState state,
    MapCommand command,
    bool success,
    const std::string & map_base,
    const std::string & message)
  {
    std::vector<uint8_t> payload;
    append_u32(payload, stamp_ms(now()));
    append_u8(payload, static_cast<uint8_t>(state));
    append_u8(payload, static_cast<uint8_t>(command));
    append_u8(payload, success ? 1U : 0U);
    append_string(payload, map_base);
    append_string(payload, message);
    enqueue(MessageType::MAP_CONTROL_STATUS, payload, FramePriority::HIGH);
  }

  void start_worker(std::function<void()> task)
  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    worker_threads_.emplace_back(std::move(task));
  }

  std::string process_snapshot()
  {
    std::string output;
    if (run_command("ps -eo args=", &output) != 0) {
      return "";
    }
    return output;
  }

  bool manual_mapping_running()
  {
    if (std::filesystem::exists("/home/bianbu/.ros/k1muse_exploration/run/remote_exploration.pid")) {
      return false;
    }
    const std::string output = process_snapshot();
    if (contains_any(
        output,
        {
          "k1muse_exploration explore.launch.py",
          "rrt_frontier_explorer",
        }))
    {
      return false;
    }
    return contains_any(
      output,
      {
        "k1muse_slam_nav slam.launch.py",
      }) ||
           std::filesystem::exists("/home/bianbu/.ros/k1muse_slam_nav/run/remote_slam_mapping.pid");
  }

  bool auto_mapping_running()
  {
    const std::string output = process_snapshot();
    return contains_any(
      output,
      {
        "k1muse_exploration explore.launch.py",
        "rrt_frontier_explorer",
        "/k1muse_exploration/lib/k1muse_exploration/rrt_frontier_explorer",
      }) ||
           std::filesystem::exists("/home/bianbu/.ros/k1muse_exploration/run/remote_exploration.pid");
  }

  bool mapping_running()
  {
    return manual_mapping_running() || auto_mapping_running();
  }

  std::string script_command(const std::string & script_name, const std::string & args = "") const
  {
    std::string command = shell_quote(mapping_scripts_dir_ + "/" + script_name);
    if (!args.empty()) {
      command += " " + args;
    }
    return command;
  }

  std::string exploration_script_command(const std::string & script_name, const std::string & args = "") const
  {
    std::string command = shell_quote(exploration_scripts_dir_ + "/" + script_name);
    if (!args.empty()) {
      command += " " + args;
    }
    return command;
  }

  std::string room_boundary_args(
    RoomSize room_size,
    float custom_size_x = 10.0F,
    float custom_size_y = 10.0F) const
  {
    switch (room_size) {
      case RoomSize::SMALL:
        return "-3 3 -3 3";
      case RoomSize::LARGE:
        return "-7.5 7.5 -7.5 7.5";
      case RoomSize::CUSTOM:
        {
          const float safe_x = std::isfinite(custom_size_x) ?
            std::clamp(custom_size_x, 1.0F, 30.0F) : 10.0F;
          const float safe_y = std::isfinite(custom_size_y) ?
            std::clamp(custom_size_y, 1.0F, 30.0F) : 10.0F;
          const float half_x = safe_x * 0.5F;
          const float half_y = safe_y * 0.5F;
          std::ostringstream args;
          args << -half_x << " " << half_x << " " << -half_y << " " << half_y;
          return args.str();
        }
      case RoomSize::MEDIUM:
      default:
        return "-5 5 -5 5";
    }
  }

  std::string stop_running_mapping_command()
  {
    std::string command;
    if (auto_mapping_running()) {
      command += exploration_script_command("k1_stop_exploration.sh", "fast");
    }
    if (manual_mapping_running()) {
      if (!command.empty()) {
        command += " ; ";
      }
      command += script_command("k1_stop_mapping.sh", "fast");
    }
    if (command.empty()) {
      command = exploration_script_command("k1_stop_exploration.sh", "fast") + " ; " +
        script_command("k1_stop_mapping.sh", "fast");
    }
    return command;
  }

  void cleanup_robot_processes(const std::string & reason)
  {
    bool expected = false;
    if (!cleanup_started_.compare_exchange_strong(expected, true)) {
      return;
    }
    RCLCPP_INFO(get_logger(), "Cleaning K1 mobile processes: %s", reason.c_str());
    publish_stop_twist();
    std::string output;
    const int rc = run_command(
      exploration_script_command("k1_stop_exploration.sh", "fast") + " ; " +
      script_command("k1_stop_mapping.sh", "fast"),
      &output);
    if (rc != 0) {
      RCLCPP_WARN(
        get_logger(), "Fast mapping cleanup failed rc=%d: %s",
        rc, last_non_empty_line(output).c_str());
    }
  }

  void shutdown_after_mobile_disconnect(const std::string & reason)
  {
    bool expected = false;
    if (!shutdown_requested_.compare_exchange_strong(expected, true)) {
      return;
    }
    RCLCPP_WARN(get_logger(), "Mobile bridge shutting down: %s", reason.c_str());
    cleanup_robot_processes(reason);
    transport_.close_device();
    rclcpp::shutdown();
    std::_Exit(0);
  }

  void handle_bridge_control(const std::vector<uint8_t> & payload)
  {
    if (payload.size() < 5U) {
      send_bridge_status(BridgeState::ERROR, BridgeCommand::QUERY_BRIDGE, false, "bad bridge control payload");
      return;
    }

    const auto command = static_cast<BridgeCommand>(payload[4]);
    switch (command) {
      case BridgeCommand::START_BRIDGE:
        send_bridge_status(BridgeState::ONLINE, command, true, "mobile bridge already online");
        break;
      case BridgeCommand::QUERY_BRIDGE:
        send_bridge_status(BridgeState::ONLINE, command, true, "mobile bridge online");
        break;
      case BridgeCommand::STOP_BRIDGE:
        send_bridge_status(BridgeState::STOPPING, command, true, "mobile bridge stopping");
        start_worker([this]() {
          cleanup_robot_processes("STOP_BRIDGE");
          send_bridge_status(BridgeState::STOPPING, BridgeCommand::STOP_BRIDGE, true, "mobile bridge cleanup done");
          std::this_thread::sleep_for(300ms);
          rclcpp::shutdown();
        });
        break;
      default:
        send_bridge_status(BridgeState::ERROR, command, false, "unknown bridge command");
        break;
    }
  }

  void handle_map_control(const std::vector<uint8_t> & payload)
  {
    if (payload.size() < 5U) {
      send_map_status(MapState::ERROR, MapCommand::QUERY_MAPPING, false, "", "bad map control payload");
      return;
    }

    const auto command = static_cast<MapCommand>(payload[4]);
    MappingMode requested_mode = MappingMode::MANUAL;
    RoomSize requested_room_size = RoomSize::MEDIUM;
    float requested_custom_size_x = 10.0F;
    float requested_custom_size_y = 10.0F;
    if (payload.size() >= 6U && payload[5] == static_cast<uint8_t>(MappingMode::AUTO)) {
      requested_mode = MappingMode::AUTO;
    }
    if (payload.size() >= 7U) {
      const auto room_raw = payload[6];
      if (room_raw == static_cast<uint8_t>(RoomSize::SMALL)) {
        requested_room_size = RoomSize::SMALL;
      } else if (room_raw == static_cast<uint8_t>(RoomSize::LARGE)) {
        requested_room_size = RoomSize::LARGE;
      } else if (room_raw == static_cast<uint8_t>(RoomSize::CUSTOM)) {
        requested_room_size = RoomSize::CUSTOM;
      }
    }
    if (requested_room_size == RoomSize::CUSTOM && payload.size() >= 15U) {
      requested_custom_size_x = read_float_le(payload, 7U);
      requested_custom_size_y = read_float_le(payload, 11U);
    }
    bool command_active = false;
    {
      std::lock_guard<std::mutex> lock(control_mutex_);
      command_active = map_command_active_;
    }
    RCLCPP_INFO(
      get_logger(),
      "Received map control command=%u active=%s",
      static_cast<unsigned>(payload[4]),
      command_active ? "true" : "false");
    if (command == MapCommand::QUERY_MAPPING) {
      {
        std::lock_guard<std::mutex> lock(control_mutex_);
        if (map_command_active_) {
          send_map_status(
            active_map_state_,
            active_map_command_,
            true,
            "",
            "mapping command in progress");
          return;
        }
      }
      const bool auto_running = auto_mapping_running();
      const bool manual_running = manual_mapping_running();
      send_map_status(
        (auto_running || manual_running) ? MapState::MAPPING : MapState::IDLE,
        command,
        true,
        "",
        auto_running ? "auto mapping is running" :
        (manual_running ? "manual mapping is running" : "mapping is idle"));
      return;
    }

    {
      std::lock_guard<std::mutex> lock(control_mutex_);
      if (map_command_active_) {
        if (active_map_command_ == command) {
          send_map_status(active_map_state_, command, true, "", "mapping command already in progress");
        } else {
          send_map_status(MapState::ERROR, command, false, "", "another mapping command is running");
        }
        return;
      }
      map_command_active_ = true;
    }

    MapState active_state = MapState::ERROR;
    std::string command_line;
    switch (command) {
      case MapCommand::START_MAPPING:
        if (mapping_running()) {
          {
            std::lock_guard<std::mutex> lock(control_mutex_);
            map_command_active_ = false;
            active_map_state_ = MapState::MAPPING;
            active_map_command_ = command;
          }
          send_map_status(MapState::MAPPING, command, true, "", "mapping is already running");
          return;
        }
        active_state = MapState::STARTING;
        command_line = requested_mode == MappingMode::AUTO ?
          exploration_script_command(
            "k1_start_exploration.sh",
            room_boundary_args(requested_room_size, requested_custom_size_x, requested_custom_size_y)) :
          script_command("k1_start_mapping.sh");
        break;
      case MapCommand::SAVE_MAP_MANUAL:
        active_state = MapState::SAVING;
        command_line = auto_mapping_running() ?
          script_command("k1_save_map.sh", "auto") :
          script_command("k1_save_map.sh", "manual");
        break;
      case MapCommand::STOP_MAPPING:
        active_state = MapState::STOPPING;
        command_line = stop_running_mapping_command();
        break;
      default:
        {
          std::lock_guard<std::mutex> lock(control_mutex_);
          map_command_active_ = false;
        }
        send_map_status(MapState::ERROR, command, false, "", "unknown mapping command");
        return;
    }

    {
      std::lock_guard<std::mutex> lock(control_mutex_);
      active_map_state_ = active_state;
      active_map_command_ = command;
    }
    send_map_status(active_state, command, true, "", "mapping command accepted");

    start_worker([this, command, command_line, active_state, requested_mode]() {
      std::string output;
      const int rc = run_command(command_line, &output);
      const std::string map_base = extract_marker(output, "MAP_BASE=");
      const std::string save_failed = extract_marker(output, "MAP_SAVE_FAILED=");
      std::string message = last_non_empty_line(output);
      if (message.empty()) {
        message = rc == 0 ? "mapping command completed" : "mapping command failed";
      }
      if (rc == 0 && command == MapCommand::START_MAPPING &&
        output.find("WARNING: no /imu/data") != std::string::npos)
      {
        message = "mapping started without IMU";
      }
      if (rc == 0 && command == MapCommand::START_MAPPING && requested_mode == MappingMode::AUTO) {
        message = "auto mapping started";
      }
      if (!save_failed.empty()) {
        message = "auto save failed; mapping stopped";
      }

      MapState final_state = MapState::ERROR;
      bool success = rc == 0;
      if (success) {
        switch (command) {
          case MapCommand::START_MAPPING:
            final_state = MapState::MAPPING;
            break;
          case MapCommand::SAVE_MAP_MANUAL:
            final_state = mapping_running() ? MapState::MAPPING : MapState::IDLE;
            break;
          case MapCommand::STOP_MAPPING:
            final_state = MapState::IDLE;
            break;
          default:
            final_state = active_state;
            break;
        }
      }
      send_map_status(final_state, command, success, map_base, message);
      {
        std::lock_guard<std::mutex> lock(control_mutex_);
        map_command_active_ = false;
        active_map_state_ = final_state;
        active_map_command_ = command;
      }
    });
  }

  void handle_mobile_frame(MessageType type, const std::vector<uint8_t> & payload)
  {
    if (type == MessageType::BRIDGE_CONTROL) {
      handle_bridge_control(payload);
      return;
    }

    if (type == MessageType::MAP_CONTROL) {
      handle_map_control(payload);
      return;
    }

    if (type == MessageType::MAP_LIBRARY_REQUEST) {
      handle_map_library_request(payload);
      return;
    }

    if (type == MessageType::STOP) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Received mobile STOP");
      publish_stop_twist();
      return;
    }

    if (type != MessageType::TELEOP_CMD) {
      return;
    }
    if (payload.size() < 16U) {
      return;
    }
    if (auto_mapping_running()) {
      publish_stop_twist();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring mobile teleop while auto mapping is running");
      return;
    }

    geometry_msgs::msg::Twist twist;
    // Payload: [stamp_ms:u32] [vx:f32] [vy:f32] [omega:f32]
    const uint32_t android_stamp_ms = read_u32_le(payload, 0U);
    twist.linear.x = read_float_le(payload, 4U);
    twist.linear.y = read_float_le(payload, 8U);
    twist.angular.z = read_float_le(payload, 12U);
    const int64_t k1_now_ms = now().nanoseconds() / 1000000LL;
    const uint32_t k1_stamp_low = static_cast<uint32_t>(k1_now_ms & 0xffffffffULL);
    const int32_t transit_ms = static_cast<int32_t>(k1_stamp_low - android_stamp_ms);
    if (transit_ms > 500) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Teleop transit %d ms (android=%u k1=%ld) — Bluetooth latency spike",
        transit_ms, android_stamp_ms, (long)k1_now_ms);
    }
    const bool was_recent_teleop = recent_teleop_active();
    last_teleop_time_ns_ = now().nanoseconds();
    teleop_active_ = true;
    const size_t purged = purge_low_priority_frames();
    if (!was_recent_teleop) {
      transport_.flush_output();
    }
    append_debug_line(
      "mobile_bridge_debug.tsv", "teleop_rx",
      "android_low_ms=" + std::to_string(android_stamp_ms) +
      "\tk1_low_ms=" + std::to_string(k1_stamp_low) +
      "\ttransit_ms=" + std::to_string(transit_ms) +
      "\tvx=" + std::to_string(twist.linear.x) +
      "\tvy=" + std::to_string(twist.linear.y) +
      "\twz=" + std::to_string(twist.angular.z) +
      "\tpurged_low=" + std::to_string(purged) +
      "\tflush=" + std::to_string(was_recent_teleop ? 0 : 1));
    cmd_vel_pub_->publish(twist);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Received mobile teleop vx=%.3f vy=%.3f wz=%.3f (transit %d ms)",
      twist.linear.x, twist.linear.y, twist.angular.z, transit_ms);
  }

  void publish_stop_twist()
  {
    geometry_msgs::msg::Twist twist;
    cmd_vel_pub_->publish(twist);
    last_teleop_time_ns_ = now().nanoseconds();
    teleop_active_ = false;
  }

  void check_teleop_timeout()
  {
    if (!teleop_active_) {
      return;
    }
    const int64_t last_ns = last_teleop_time_ns_.load();
    const int elapsed_ms = static_cast<int>((now().nanoseconds() - last_ns) / 1000000LL);
    if (elapsed_ms > teleop_timeout_ms_) {
      publish_stop_twist();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Mobile teleop timeout (%d ms > %d ms), publishing zero /cmd_vel",
        elapsed_ms, teleop_timeout_ms_);
    }
  }

  void rx_loop()
  {
    std::vector<uint8_t> buffer(1024);
    for (;;) {
      {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (stopping_) {
          return;
        }
      }

      if (!transport_.is_open()) {
        std::this_thread::sleep_for(100ms);
        continue;
      }

      const ssize_t n = transport_.read_some(buffer.data(), buffer.size(), 100);
      if (n < 0) {
        shutdown_after_mobile_disconnect("mobile RFCOMM read failed or disconnected");
        return;
      }
      if (n == 0) {
        continue;
      }
      rx_buffer_.insert(rx_buffer_.end(), buffer.begin(), buffer.begin() + n);
      parse_rx_buffer();
    }
  }

  void parse_rx_buffer()
  {
    constexpr size_t header_size = 20U;
    constexpr size_t max_payload_size = 4U * 1024U * 1024U;
    constexpr uint8_t magic[4] = {'K', '1', 'M', 'B'};

    for (;;) {
      auto start = std::search(rx_buffer_.begin(), rx_buffer_.end(), std::begin(magic), std::end(magic));
      if (start == rx_buffer_.end()) {
        rx_buffer_.clear();
        return;
      }
      if (start != rx_buffer_.begin()) {
        rx_buffer_.erase(rx_buffer_.begin(), start);
      }
      if (rx_buffer_.size() < header_size) {
        return;
      }

      const uint8_t version = rx_buffer_[4];
      const uint8_t type_raw = rx_buffer_[5];
      (void)read_u16_le(rx_buffer_, 6U);
      const uint32_t payload_len = read_u32_le(rx_buffer_, 12U);
      const uint32_t expected_crc = read_u32_le(rx_buffer_, 16U);
      if (version != 1U || payload_len > max_payload_size) {
        rx_buffer_.erase(rx_buffer_.begin());
        continue;
      }
      const size_t total = header_size + payload_len;
      if (rx_buffer_.size() < total) {
        return;
      }

      std::vector<uint8_t> payload(rx_buffer_.begin() + header_size, rx_buffer_.begin() + total);
      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + total);
      if (payload_crc(payload) != expected_crc) {
        continue;
      }

      const auto type = static_cast<MessageType>(type_raw);
      handle_mobile_frame(type, payload);
    }
  }

  bool recent_teleop_active() const
  {
    const int64_t last_ns = last_teleop_time_ns_.load();
    return last_ns > 0 && (now().nanoseconds() - last_ns) < 1500000000LL;
  }

  size_t purge_low_priority_frames()
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    const size_t before = tx_queue_.size();
    tx_queue_.erase(
      std::remove_if(tx_queue_.begin(), tx_queue_.end(), [](const QueuedFrame & item) {
        return item.priority == FramePriority::LOW;
      }),
      tx_queue_.end());
    return before - tx_queue_.size();
  }

  bool enqueue(MessageType type, const std::vector<uint8_t> & payload, FramePriority priority)
  {
    std::vector<uint8_t> frame;
    try {
      frame = encode_frame(type, next_seq_++, payload);
    } catch (const std::exception & ex) {
      RCLCPP_WARN(get_logger(), "Failed to encode mobile frame: %s", ex.what());
      return false;
    }

    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (type == MessageType::ROBOT_POSE) {
      // ponytail: live telemetry is latest-only; stale poses just create Bluetooth lag.
      tx_queue_.erase(
        std::remove_if(tx_queue_.begin(), tx_queue_.end(), [](const QueuedFrame & item) {
          return item.type == MessageType::ROBOT_POSE;
        }),
        tx_queue_.end());
    }
    if (priority == FramePriority::LOW && recent_teleop_active()) {
      tx_queue_.erase(
        std::remove_if(tx_queue_.begin(), tx_queue_.end(), [](const QueuedFrame & item) {
          return item.priority == FramePriority::LOW;
        }),
        tx_queue_.end());
      append_debug_line(
        "mobile_bridge_debug.tsv", "enqueue_drop",
        std::string("type=") + message_type_name(type) +
        "\treason=teleop_active");
      return false;
    }
    if (priority == FramePriority::LOW) {
      const auto low_count = std::count_if(tx_queue_.begin(), tx_queue_.end(), [](const QueuedFrame & item) {
        return item.priority == FramePriority::LOW;
      });
      if (low_count >= max_low_queue_frames_) {
        append_debug_line(
          "mobile_bridge_debug.tsv", "enqueue_drop",
          std::string("type=") + message_type_name(type) +
          "\treason=low_queue_full\tlow_count=" + std::to_string(low_count));
        return false;
      }
    }
    while (tx_queue_.size() >= static_cast<size_t>(max_queue_frames_)) {
      if (priority == FramePriority::LOW) {
        return false;
      }
      auto it = std::find_if(tx_queue_.begin(), tx_queue_.end(), [](const QueuedFrame & item) {
        return item.priority == FramePriority::LOW;
      });
      if (it != tx_queue_.end()) {
        tx_queue_.erase(it);
      } else {
        tx_queue_.pop_front();
      }
    }

    auto insert_at = std::find_if(tx_queue_.begin(), tx_queue_.end(), [priority](const QueuedFrame & item) {
      return static_cast<uint8_t>(item.priority) > static_cast<uint8_t>(priority);
    });
    tx_queue_.insert(insert_at, QueuedFrame{type, priority, std::move(frame)});
    if (type == MessageType::MAP_TILE || type == MessageType::TELEOP_CMD || type == MessageType::ROBOT_POSE) {
      append_debug_line(
        "mobile_bridge_debug.tsv", "enqueue",
        std::string("type=") + message_type_name(type) +
        "\tpriority=" + std::to_string(static_cast<int>(priority)) +
        "\tqueue=" + std::to_string(tx_queue_.size()));
    }
    queue_cv_.notify_one();
    return true;
  }

  void tx_loop()
  {
    for (;;) {
      QueuedFrame item;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cv_.wait(lock, [this]() {
          return stopping_ || !tx_queue_.empty();
        });
        if (stopping_) {
          return;
        }
        item = std::move(tx_queue_.front());
        tx_queue_.pop_front();
      }

      if (!transport_.is_open()) {
        if (item.priority != FramePriority::LOW) {
          std::lock_guard<std::mutex> lock(queue_mutex_);
          tx_queue_.push_front(std::move(item));
        }
        std::this_thread::sleep_for(100ms);
        continue;
      }

      if (item.priority == FramePriority::LOW && recent_teleop_active()) {
        append_debug_line(
          "mobile_bridge_debug.tsv", "tx_skip",
          std::string("type=") + message_type_name(item.type) + "\treason=teleop_active");
        continue;
      }
      if (!transport_.write_all(item.frame)) {
        if (!transport_.is_open()) {
          shutdown_after_mobile_disconnect("mobile RFCOMM write failed or disconnected");
          return;
        }
        const auto failed_type = item.type;
        const auto failed_priority = item.priority;
        const auto failed_bytes = item.frame.size();
        // Non-blocking fd: kernel buffer full (EAGAIN).  Drop LOW frames,
        // requeue NORMAL frames so essential data is not lost.
        if (item.priority != FramePriority::LOW) {
          std::lock_guard<std::mutex> lock(queue_mutex_);
          tx_queue_.push_front(std::move(item));
        }
        append_debug_line(
          "mobile_bridge_debug.tsv", "tx_fail",
          std::string("type=") + message_type_name(failed_type) +
          "\tpriority=" + std::to_string(static_cast<int>(failed_priority)) +
          "\tbytes=" + std::to_string(failed_bytes));
        // brief backoff before next write attempt
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }
      if (item.type == MessageType::MAP_TILE || item.type == MessageType::ROBOT_POSE ||
        item.type == MessageType::MAP_INFO)
      {
        append_debug_line(
          "mobile_bridge_debug.tsv", "tx_ok",
          std::string("type=") + message_type_name(item.type) +
          "\tpriority=" + std::to_string(static_cast<int>(item.priority)) +
          "\tbytes=" + std::to_string(item.frame.size()));
      }
    }
  }

  std::string rfcomm_device_;
  std::string map_topic_;
  std::string global_frame_;
  std::string base_frame_;
  std::string cmd_vel_topic_;
  std::string mapping_scripts_dir_;
  std::string exploration_scripts_dir_;
  std::string exploration_path_topic_;
  std::string map_library_dir_;
  int tile_size_ = 64;
  int max_queue_frames_ = 256;
  int max_low_queue_frames_ = 8;
  int max_map_tiles_per_update_ = 8;
  int teleop_timeout_ms_ = 300;
  double map_publish_hz_ = 1.0;
  double pose_publish_hz_ = 10.0;
  double path_publish_hz_ = 1.0;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::TimerBase::SharedPtr connect_timer_;
  rclcpp::TimerBase::SharedPtr pose_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr teleop_timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  RfcommTransport transport_;

  rclcpp::Time last_map_tx_;
  rclcpp::Time last_path_tx_;
  uint32_t map_version_ = 0;
  uint32_t path_version_ = 0;
  bool map_info_sent_ = false;
  uint32_t map_width_ = 0;
  uint32_t map_height_ = 0;
  float map_resolution_ = 0.0F;
  float map_origin_x_ = 0.0F;
  float map_origin_y_ = 0.0F;
  float map_origin_yaw_ = 0.0F;
  std::vector<uint32_t> tile_hash_;

  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<QueuedFrame> tx_queue_;
  std::vector<uint8_t> rx_buffer_;
  std::thread tx_thread_;
  std::thread rx_thread_;
  std::mutex worker_mutex_;
  std::vector<std::thread> worker_threads_;
  std::mutex control_mutex_;
  std::atomic_bool cleanup_started_{false};
  std::atomic_bool shutdown_requested_{false};
  bool map_command_active_ = false;
  MapState active_map_state_ = MapState::IDLE;
  MapCommand active_map_command_ = MapCommand::QUERY_MAPPING;
  bool stopping_ = false;
  uint32_t next_seq_ = 1;
  std::atomic<int64_t> last_teleop_time_ns_{0};
  std::atomic_bool teleop_active_{false};
};

}  // namespace k1muse_mobile_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<k1muse_mobile_bridge::MobileBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
