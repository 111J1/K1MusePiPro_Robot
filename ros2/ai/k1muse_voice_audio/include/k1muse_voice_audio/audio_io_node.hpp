#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "k1muse_common/qos_profiles.hpp"
#include "k1muse_voice_audio/audio_backend.hpp"
#include "k1muse_voice_audio/audio_device.hpp"
#include "k1muse_voice_msgs/msg/audio_play_request.hpp"
#include "k1muse_voice_msgs/msg/audio_playback_state.hpp"
#include "k1muse_voice_msgs/msg/tts_play_request.hpp"
#include "k1muse_audio_msgs/msg/audio_frame.hpp"
#include "k1muse_common/msg/node_ready.hpp"

namespace k1muse_voice_audio
{

class AudioIoNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using TtsPlayRequestMessage = k1muse_voice_msgs::msg::TtsPlayRequest;
  using AudioPlayRequestMessage = k1muse_voice_msgs::msg::AudioPlayRequest;
  using AudioPlaybackStateMessage = k1muse_voice_msgs::msg::AudioPlaybackState;
  using NodeReadyMessage = k1muse_common::msg::NodeReady;

  explicit AudioIoNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~AudioIoNode() override;

  // Testing helpers
  bool is_ready() const noexcept;
  std::string last_error() const;
  std::size_t queue_size() const;
  bool worker_running() const noexcept;

  // Inject a custom device for testing (call after configure, before activate).
  void set_device_for_testing(std::unique_ptr<AudioDevice> device);

protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  struct PlaybackRequest
  {
    std::string trace_id;
    std::string request_id;
    uint64_t epoch{0};
    std::string source;
    bool is_preset{false};
    std::string preset_name;
    uint32_t sample_rate{0};
    uint8_t channels{0};
    std::string encoding;
    std::vector<int16_t> pcm;
  };

  void on_tts_request(const TtsPlayRequestMessage::SharedPtr message);
  void on_audio_request(const AudioPlayRequestMessage::SharedPtr message);

  bool enqueue_request(PlaybackRequest request);
  void worker_loop();

  // Capture (ALSA / PortAudio input via AudioBackend)
  void start_capture();
  void stop_capture();
  void capture_loop();
  bool init_capture_backend();
  bool attempt_capture_reconnect();

  void publish_playback_state(
    const std::string & trace_id, const std::string & request_id,
    uint64_t epoch, const std::string & source, uint8_t state,
    const std::string & state_name, const std::string & reason = {});
  void publish_ready(bool ready, const std::string & reason = {});
  void stop_worker();
  void set_last_error(const std::string & error);

  std::string device_type_;
  std::unique_ptr<AudioDevice> device_;

  // Capture config
  std::string capture_device_;
  uint32_t capture_sample_rate_{16000};
  uint8_t capture_channels_{1};
  uint16_t capture_frame_ms_{20};
  uint32_t capture_buffer_ms_{400};

  // Playback config
  std::string playback_device_;
  std::string preset_wake_ack_path_;

  rclcpp::CallbackGroup::SharedPtr request_callback_group_;
  rclcpp::Publisher<AudioPlaybackStateMessage>::SharedPtr playback_state_publisher_;
  rclcpp::Publisher<NodeReadyMessage>::SharedPtr ready_publisher_;
  rclcpp::Publisher<k1muse_audio_msgs::msg::AudioFrame>::SharedPtr audio_publisher_;
  rclcpp::Subscription<TtsPlayRequestMessage>::SharedPtr tts_subscription_;
  rclcpp::Subscription<AudioPlayRequestMessage>::SharedPtr audio_subscription_;

  // Playback queue (capacity 1; wake_ack and TTS never overlap)
  static constexpr std::size_t kMaxQueueSize = 1;
  std::queue<PlaybackRequest> playback_queue_;
  mutable std::mutex queue_mutex_;
  std::string last_audio_req_id_;  // Deduplicate DDS redelivery
  std::condition_variable queue_cv_;

  std::atomic<bool> worker_running_{false};
  std::thread worker_thread_;

  // Capture (via AudioBackend — unified PortAudio on K1)
  std::unique_ptr<AudioBackend> capture_backend_;
  AudioConfig capture_config_;
  std::atomic<bool> capture_running_{false};
  std::atomic<bool> capture_faulted_{false};
  std::thread capture_thread_;
  std::atomic<uint32_t> capture_seq_{0};
  mutable std::mutex capture_trace_mutex_;
  std::string capture_trace_id_{"audio_io"};  // Synced to supervisor on wakeword

  // Fault recovery
  mutable std::mutex capture_error_mutex_;
  int capture_consecutive_errors_{0};
  int max_reconnect_attempts_{10};
  int reconnect_backoff_ms_{100};
  int reconnect_max_backoff_ms_{5000};

  mutable std::mutex error_mutex_;
  std::string last_error_;
  std::atomic<bool> ready_{false};
};

}  // namespace k1muse_voice_audio
