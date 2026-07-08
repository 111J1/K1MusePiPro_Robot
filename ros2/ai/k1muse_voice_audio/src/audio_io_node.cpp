#include "k1muse_voice_audio/audio_io_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <fstream>
#include <functional>
#include <utility>

#include <portaudio.h>

#include "k1muse_voice_audio/mock_audio_device.hpp"
#include "k1muse_voice_audio/portaudio_backend.hpp"
#include "k1muse_voice_audio/portaudio_device.hpp"

namespace k1muse_voice_audio
{

namespace
{
// Load a WAV file and return PCM data + sample rate.
std::vector<int16_t> load_wav_pcm(
    const std::string & path, int & out_sample_rate, std::string * error)
{
  std::ifstream file(path, std::ios::binary | std::ios::ate);
  if (!file.is_open()) {
    if (error) *error = "cannot open: " + path;
    return {};
  }
  std::streamsize size = file.tellg();
  file.seekg(0, std::ios::beg);
  if (size < 44) {
    if (error) *error = "WAV too short";
    return {};
  }
  std::vector<char> raw(static_cast<size_t>(size));
  if (!file.read(raw.data(), size)) {
    if (error) *error = "cannot read: " + path;
    return {};
  }

  uint32_t wav_sr = 0, data_sz = 0;
  uint16_t wav_ch = 0, wav_bps = 0;
  std::memcpy(&wav_ch, raw.data() + 22, sizeof(wav_ch));
  std::memcpy(&wav_sr, raw.data() + 24, sizeof(wav_sr));
  std::memcpy(&wav_bps, raw.data() + 34, sizeof(wav_bps));
  std::memcpy(&data_sz, raw.data() + 40, sizeof(data_sz));

  if (wav_bps != 16) {
    if (error) *error = "WAV not 16-bit";
    return {};
  }
  out_sample_rate = static_cast<int>(wav_sr);
  size_t sample_count = data_sz / 2;
  const auto * samples = reinterpret_cast<const int16_t *>(raw.data() + 44);
  return std::vector<int16_t>(samples, samples + sample_count);
}
}  // namespace

AudioIoNode::AudioIoNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("audio_io", options)
{
  // Declare parameters with defaults
  this->declare_parameter<std::string>("device_type", "mock");

  // Capture parameters (used by "alsa" device type)
  this->declare_parameter<std::string>("capture_device", "");
  this->declare_parameter<int>("capture_sample_rate", 16000);
  this->declare_parameter<int>("capture_channels", 1);
  this->declare_parameter<int>("capture_frame_ms", 20);
  this->declare_parameter<int>("capture_buffer_ms", 400);

  // Playback parameters
  this->declare_parameter<std::string>("playback_device", "");
  this->declare_parameter<std::string>("preset_wake_ack_path",
      "/home/bianbu/.cache/assets/audio/wozai.wav");
}

AudioIoNode::~AudioIoNode()
{
  stop_worker();
}

bool AudioIoNode::is_ready() const noexcept
{
  return ready_.load();
}

std::string AudioIoNode::last_error() const
{
  std::lock_guard<std::mutex> lock(error_mutex_);
  return last_error_;
}

std::size_t AudioIoNode::queue_size() const
{
  std::lock_guard<std::mutex> lock(queue_mutex_);
  return playback_queue_.size();
}

bool AudioIoNode::worker_running() const noexcept
{
  return worker_running_.load();
}

void AudioIoNode::set_device_for_testing(std::unique_ptr<AudioDevice> device)
{
  device_ = std::move(device);
}

// --- Lifecycle callbacks ---

AudioIoNode::CallbackReturn AudioIoNode::on_configure(
  const rclcpp_lifecycle::State & /*state*/)
{
  device_type_ = this->get_parameter("device_type").as_string();

  // Read capture parameters
  capture_device_ = this->get_parameter("capture_device").as_string();
  capture_sample_rate_ = static_cast<uint32_t>(this->get_parameter("capture_sample_rate").as_int());
  capture_channels_ = static_cast<uint8_t>(this->get_parameter("capture_channels").as_int());
  capture_frame_ms_ = static_cast<uint16_t>(this->get_parameter("capture_frame_ms").as_int());
  capture_buffer_ms_ = static_cast<uint32_t>(this->get_parameter("capture_buffer_ms").as_int());

  // Read playback parameters
  playback_device_ = this->get_parameter("playback_device").as_string();
  preset_wake_ack_path_ = this->get_parameter("preset_wake_ack_path").as_string();

  // Create playback facade only for mock and explicit PortAudio modes.
  // ALSA uses one AudioBackend instance for both capture and playback so
  // plughw devices are opened deterministically instead of via name matching.
  if (device_type_ == "mock") {
    device_ = std::make_unique<MockAudioDevice>();
  } else if (device_type_ == "portaudio") {
    PortAudioDevice::Config pa_cfg;
    pa_cfg.playback_device = playback_device_;
    pa_cfg.preset_wake_ack_path = preset_wake_ack_path_;
    pa_cfg.sample_rate = capture_sample_rate_;
    pa_cfg.channels = capture_channels_;
    pa_cfg.frames_per_buffer = capture_sample_rate_ * capture_frame_ms_ / 1000;
    device_ = std::make_unique<PortAudioDevice>(std::move(pa_cfg));
  } else if (device_type_ != "alsa") {
    set_last_error("unknown device_type: " + device_type_);
    return CallbackReturn::FAILURE;
  }

  if (device_) {
    device_->load();
  }

  // Init capture/playback backend.
  if (!init_capture_backend()) {
    return CallbackReturn::FAILURE;
  }

  // Create callback group for subscriptions
  request_callback_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  // Create publishers
  playback_state_publisher_ = this->create_publisher<AudioPlaybackStateMessage>(
    "/voice/audio/playback_state", k1muse_common::qos::LatchedState(1));

  ready_publisher_ = this->create_publisher<NodeReadyMessage>(
    "/audio_io/state", k1muse_common::qos::LatchedState(1));

  // Create audio capture publisher (publishes /audio/raw_pcm)
  audio_publisher_ = this->create_publisher<k1muse_audio_msgs::msg::AudioFrame>(
    "/audio/raw_pcm", k1muse_common::qos::AudioStream(20));

  // Create subscriptions with callback group
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = request_callback_group_;

  tts_subscription_ = this->create_subscription<TtsPlayRequestMessage>(
    "/voice/tts/play", k1muse_common::qos::ReliableEvent(5),
    std::bind(&AudioIoNode::on_tts_request, this, std::placeholders::_1),
    sub_options);

  audio_subscription_ = this->create_subscription<AudioPlayRequestMessage>(
    "/voice/audio/play", k1muse_common::qos::ReliableEvent(5),
    std::bind(&AudioIoNode::on_audio_request, this, std::placeholders::_1),
    sub_options);

  publish_ready(false, "configured");

  RCLCPP_INFO(get_logger(),
    "[startup] audio_io device_type=%s capture_device=%s playback_device=%s "
    "capture=%uHz/%uch/%ums buffer_ms=%u preset_wake_ack=%s "
    "topics={audio_out:/audio/raw_pcm tts_in:/voice/tts/play "
    "audio_in:/voice/audio/play playback_state:/voice/audio/playback_state "
    "state:/audio_io/state}",
    device_type_.c_str(), capture_device_.c_str(), playback_device_.c_str(),
    capture_sample_rate_, capture_channels_, capture_frame_ms_,
    capture_buffer_ms_, preset_wake_ack_path_.c_str());
  return CallbackReturn::SUCCESS;
}

AudioIoNode::CallbackReturn AudioIoNode::on_activate(
  const rclcpp_lifecycle::State & /*state*/)
{
  // Start playback worker thread
  worker_running_ = true;
  worker_thread_ = std::thread(&AudioIoNode::worker_loop, this);

  // Start capture for real backends.
  if (device_type_ != "mock") {
    start_capture();
  }

  ready_ = true;
  publish_ready(true);

  RCLCPP_INFO(get_logger(), "AudioIoNode activated");
  return CallbackReturn::SUCCESS;
}

AudioIoNode::CallbackReturn AudioIoNode::on_deactivate(
  const rclcpp_lifecycle::State & /*state*/)
{
  ready_ = false;
  publish_ready(false, "deactivated");

  stop_capture();
  stop_worker();

  RCLCPP_INFO(get_logger(), "AudioIoNode deactivated");
  return CallbackReturn::SUCCESS;
}

AudioIoNode::CallbackReturn AudioIoNode::on_cleanup(
  const rclcpp_lifecycle::State & /*state*/)
{
  // Release device
  if (device_) {
    device_->unload();
    device_.reset();
  }
  // Release capture backend
  if (capture_backend_) {
    capture_backend_->close();
    capture_backend_.reset();
  }

  // Release publishers and subscriptions
  tts_subscription_.reset();
  audio_subscription_.reset();
  playback_state_publisher_.reset();
  ready_publisher_.reset();
  audio_publisher_.reset();
  request_callback_group_.reset();

  RCLCPP_INFO(get_logger(), "AudioIoNode cleaned up");
  return CallbackReturn::SUCCESS;
}

AudioIoNode::CallbackReturn AudioIoNode::on_error(
  const rclcpp_lifecycle::State & /*state*/)
{
  ready_ = false;
  publish_ready(false, "error: " + last_error());
  stop_worker();
  return CallbackReturn::SUCCESS;
}

AudioIoNode::CallbackReturn AudioIoNode::on_shutdown(
  const rclcpp_lifecycle::State & /*state*/)
{
  ready_ = false;
  publish_ready(false, "shutdown");
  stop_worker();

  if (device_) {
    device_->unload();
    device_.reset();
  }
  if (capture_backend_) {
    capture_backend_->close();
    capture_backend_.reset();
  }

  return CallbackReturn::SUCCESS;
}

// --- Subscription callbacks ---

void AudioIoNode::on_tts_request(const TtsPlayRequestMessage::SharedPtr message)
{
  PlaybackRequest request;
  request.trace_id = message->trace_id;
  request.request_id = message->request_id;
  request.epoch = message->epoch;
  request.source = message->source;
  request.is_preset = false;
  request.sample_rate = message->sample_rate;
  request.channels = message->channels;
  request.encoding = message->encoding;
  request.pcm = message->pcm_s16le;

  RCLCPP_INFO(get_logger(),
    "[trace] playback_request trace_id=%s request_id=%s epoch=%llu source=%s "
    "kind=tts sample_rate=%u channels=%u bytes=%zu",
    message->trace_id.c_str(), message->request_id.c_str(),
    static_cast<unsigned long long>(message->epoch), message->source.c_str(),
    message->sample_rate, message->channels, message->pcm_s16le.size());


  if (!enqueue_request(std::move(request))) {
    publish_playback_state(
      message->trace_id, message->request_id, message->epoch, message->source,
      AudioPlaybackStateMessage::STATE_FAILED, "FAILED", "queue full");
  }
}

void AudioIoNode::on_audio_request(const AudioPlayRequestMessage::SharedPtr message)
{
  // Deduplicate: DDS RELIABLE may redeliver the same AudioPlayRequest.
  // Only process if request_id differs from the last one we handled.
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (message->request_id == last_audio_req_id_) {
      RCLCPP_DEBUG(get_logger(), "Skipping duplicate AudioPlayRequest: %s",
                   message->request_id.c_str());
      return;
    }
    last_audio_req_id_ = message->request_id;
  }

  // Sync capture trace_id to supervisor's active_trace_id_
  // (supervisor generates a new trace_id on wakeword, replacing the
  // hardcoded "audio_io" - needed for ListenResult trace_id matching)
  if (!message->trace_id.empty()) {
    std::lock_guard<std::mutex> lock(capture_trace_mutex_);
    capture_trace_id_ = message->trace_id;
  }

  PlaybackRequest request;
  request.trace_id = message->trace_id;
  request.request_id = message->request_id;
  request.epoch = message->epoch;
  request.source = message->source;
  request.is_preset = (message->kind == AudioPlayRequestMessage::KIND_PRESET);
  request.preset_name = message->preset_name;
  request.sample_rate = message->sample_rate;
  request.channels = message->channels;
  request.encoding = message->encoding;
  request.pcm = message->pcm_s16le;

  RCLCPP_INFO(get_logger(),
    "[trace] playback_request trace_id=%s request_id=%s epoch=%llu source=%s "
    "kind=%s preset=%s sample_rate=%u channels=%u bytes=%zu",
    message->trace_id.c_str(), message->request_id.c_str(),
    static_cast<unsigned long long>(message->epoch), message->source.c_str(),
    request.is_preset ? "preset" : "pcm", request.preset_name.c_str(),
    request.sample_rate, request.channels, request.pcm.size());

  if (!enqueue_request(std::move(request))) {
    publish_playback_state(
      message->trace_id, message->request_id, message->epoch, message->source,
      AudioPlaybackStateMessage::STATE_FAILED, "FAILED", "queue full");
  }
}

// --- Queue management ---

bool AudioIoNode::enqueue_request(PlaybackRequest request)
{
  std::lock_guard<std::mutex> lock(queue_mutex_);
  if (playback_queue_.size() >= kMaxQueueSize) {
    RCLCPP_WARN(get_logger(), "Playback queue full, rejecting request %s",
      request.request_id.c_str());
    return false;
  }
  playback_queue_.push(std::move(request));
  queue_cv_.notify_one();
  return true;
}

// --- Playback worker ---

void AudioIoNode::worker_loop()
{
  RCLCPP_INFO(get_logger(), "Playback worker started");

  while (worker_running_.load()) {
    PlaybackRequest request;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(lock, [this]() {
        return !playback_queue_.empty() || !worker_running_.load();
      });

      if (!worker_running_.load()) {
        break;
      }

      if (playback_queue_.empty()) {
        continue;
      }

      request = std::move(playback_queue_.front());
      playback_queue_.pop();
    }

    // Publish PENDING state
    publish_playback_state(
      request.trace_id, request.request_id, request.epoch, request.source,
      AudioPlaybackStateMessage::STATE_PENDING, "PENDING");

    // Publish PLAYING state
    publish_playback_state(
      request.trace_id, request.request_id, request.epoch, request.source,
      AudioPlaybackStateMessage::STATE_PLAYING, "PLAYING");

    // Playback: unified through capture_backend_->write_chunk() on real K1,
    // or device_ for mock mode.  This avoids creating conflicting PortAudio
    // streams (ref: bianbu_new AudioIoComponent::consume_playback_queue).
    bool playback_ok = false;
    std::string playback_error;
    std::vector<int16_t> pcm_to_play = request.pcm;
    int out_sample_rate = static_cast<int>(request.sample_rate);

    if (request.is_preset && device_type_ != "mock") {
      // Real K1: load WAV file into PCM, then write via backend.
      pcm_to_play = load_wav_pcm(preset_wake_ack_path_, out_sample_rate, &playback_error);
      if (pcm_to_play.empty() && playback_error.empty()) {
        playback_error = "failed to load WAV: " + preset_wake_ack_path_;
      }
    }

    if (device_type_ == "mock") {
      AudioDevice::PlaybackResult result;
      if (request.is_preset) {
        result = device_->play_preset(request.preset_name);
      } else {
        result = device_->play_pcm(
            request.pcm, request.sample_rate, request.channels, request.encoding);
      }
      playback_ok = result.success;
      playback_error = result.reason;
    } else if (!pcm_to_play.empty() && capture_backend_) {
      playback_ok = capture_backend_->write_chunk(
          pcm_to_play, out_sample_rate, &playback_error);
    } else if (pcm_to_play.empty()) {
      playback_error = "empty PCM data";
    }

    if (playback_ok) {
      publish_playback_state(
        request.trace_id, request.request_id, request.epoch, request.source,
        AudioPlaybackStateMessage::STATE_DONE, "DONE");
    } else {
      if (playback_error.empty()) playback_error = "playback failed";
      publish_playback_state(
        request.trace_id, request.request_id, request.epoch, request.source,
        AudioPlaybackStateMessage::STATE_FAILED, "FAILED", playback_error);
    }
  }

  RCLCPP_INFO(get_logger(), "Playback worker stopped");
}

// --- Publishing helpers ---

void AudioIoNode::publish_playback_state(
  const std::string & trace_id, const std::string & request_id,
  uint64_t epoch, const std::string & source, uint8_t state,
  const std::string & state_name, const std::string & reason)
{
  auto message = AudioPlaybackStateMessage();
  message.header.stamp = this->now();
  message.trace_id = trace_id;
  message.request_id = request_id;
  message.epoch = epoch;
  message.source = source;
  message.state = state;
  message.state_name = state_name;
  message.reason = reason;

  if (playback_state_publisher_) {
    playback_state_publisher_->publish(message);
  }
}

void AudioIoNode::publish_ready(bool ready, const std::string & reason)
{
  auto message = NodeReadyMessage();
  message.header.stamp = this->now();
  message.node_name = "audio_io";
  message.ready = ready;
  message.reason = reason;

  if (ready_publisher_) {
    ready_publisher_->publish(message);
  }
  RCLCPP_INFO(get_logger(), "[health] audio_io ready=%d reason=%s",
    ready, reason.c_str());
}

void AudioIoNode::stop_worker()
{
  worker_running_ = false;
  queue_cv_.notify_all();

  // Also stop any ongoing device playback
  if (device_) {
    device_->stop();
  }

  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

void AudioIoNode::set_last_error(const std::string & error)
{
  std::lock_guard<std::mutex> lock(error_mutex_);
  last_error_ = error;
}

// --- Capture (ALSA / PortAudio input) ---

void AudioIoNode::start_capture()
{
  if (capture_running_.load()) return;

  capture_seq_ = 0;
  capture_running_ = true;
  capture_thread_ = std::thread(&AudioIoNode::capture_loop, this);

  RCLCPP_INFO(get_logger(), "Capture started: device=%s %uHz %uch %ums",
    capture_device_.c_str(), capture_sample_rate_,
    capture_channels_, capture_frame_ms_);
}

void AudioIoNode::stop_capture()
{
  capture_running_ = false;
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }
  RCLCPP_INFO(get_logger(), "Capture stopped");
}

void AudioIoNode::capture_loop()
{
  const bool needs_pacing =
    capture_backend_ && capture_backend_->requires_external_pacing();
  const auto sleep_duration = std::chrono::milliseconds(capture_config_.chunk_ms);
  int backoff_ms = reconnect_backoff_ms_;

  while (capture_running_.load() && rclcpp::ok()) {
    if (capture_faulted_.load(std::memory_order_acquire)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      continue;
    }

    std::vector<int16_t> samples;
    std::string error;
    if (capture_backend_ && capture_backend_->read_chunk(&samples, &error)) {
      // Successful read - reset error state.
      {
        std::lock_guard<std::mutex> lock(capture_error_mutex_);
        if (capture_consecutive_errors_ > 0) {
          RCLCPP_INFO(get_logger(),
            "Audio capture recovered after %d consecutive errors",
            capture_consecutive_errors_);
        }
        capture_consecutive_errors_ = 0;
      }
      backoff_ms = reconnect_backoff_ms_;

      auto msg = k1muse_audio_msgs::msg::AudioFrame();
      msg.header.stamp = this->now();
      msg.header.frame_id = "audio_io_capture";
      {
        std::lock_guard<std::mutex> lock(capture_trace_mutex_);
        msg.trace_id = capture_trace_id_;
      }
      msg.seq = ++capture_seq_;
      msg.sample_rate = static_cast<uint32_t>(capture_config_.sample_rate);
      msg.channels = static_cast<uint8_t>(capture_config_.channels);
      msg.encoding = "s16le";
      msg.frame_ms = static_cast<uint16_t>(capture_config_.chunk_ms);
      msg.pcm_s16le = std::move(samples);

      if (audio_publisher_) {
        audio_publisher_->publish(msg);
      }
    } else {
      // Read failed - increment error counter.
      int errors = 0;
      {
        std::lock_guard<std::mutex> lock(capture_error_mutex_);
        ++capture_consecutive_errors_;
        errors = capture_consecutive_errors_;
      }

      RCLCPP_WARN(get_logger(),
        "Audio capture read failed (consecutive errors: %d): %s",
        errors, error.c_str());

      if (max_reconnect_attempts_ >= 0 && errors > max_reconnect_attempts_) {
        RCLCPP_ERROR(get_logger(),
          "Audio capture entered fault state after %d consecutive errors", errors);
        capture_faulted_.store(true, std::memory_order_release);
        continue;
      }

      if (attempt_capture_reconnect()) {
        RCLCPP_INFO(get_logger(), "Audio capture reconnected successfully");
        {
          std::lock_guard<std::mutex> lock(capture_error_mutex_);
          capture_consecutive_errors_ = 0;
        }
        backoff_ms = reconnect_backoff_ms_;
      } else {
        RCLCPP_WARN(get_logger(),
          "Audio capture reconnection failed, backing off %d ms", backoff_ms);
        std::this_thread::sleep_for(std::chrono::milliseconds(backoff_ms));
        backoff_ms = std::min(backoff_ms * 2, reconnect_max_backoff_ms_);
      }
    }

    if (needs_pacing && !capture_faulted_.load(std::memory_order_acquire)) {
      std::this_thread::sleep_for(sleep_duration);
    }
  }

  if (capture_backend_) {
    capture_backend_->close();
  }
  RCLCPP_INFO(get_logger(), "Capture loop ended (seq=%u)", capture_seq_.load());
}

bool AudioIoNode::init_capture_backend()
{
  if (device_type_ == "mock") {
    capture_backend_ = create_fake_audio_backend();
  } else if (device_type_ == "alsa") {
    capture_backend_ = create_alsa_audio_backend();
  } else if (device_type_ == "portaudio") {
    capture_backend_ = create_portaudio_audio_backend();
  } else {
    set_last_error("unknown device_type for capture: " + device_type_);
    return false;
  }

  capture_config_.sample_rate = static_cast<int>(capture_sample_rate_);
  capture_config_.channels = static_cast<int>(capture_channels_);
  capture_config_.chunk_ms = static_cast<int>(capture_frame_ms_);
  capture_config_.capture_device = capture_device_;
  capture_config_.playback_device = playback_device_;

  std::string error;
  if (!capture_backend_->open_capture(capture_config_, &error)) {
    RCLCPP_ERROR(get_logger(), "Failed to open capture backend: %s", error.c_str());
    set_last_error("capture open failed: " + error);
    return false;
  }
  // Open playback on the same backend - avoids conflicting real-device streams.
  if (!capture_backend_->open_playback(capture_config_, &error)) {
    capture_backend_->close();
    RCLCPP_ERROR(get_logger(), "Failed to open playback backend: %s", error.c_str());
    set_last_error("playback open failed: " + error);
    return false;
  }

  RCLCPP_INFO(get_logger(), "Capture+playback backend ready: %s",
    device_type_.c_str());
  return true;
}

bool AudioIoNode::attempt_capture_reconnect()
{
  if (!capture_backend_) return false;

  std::string error;
  if (capture_backend_->reconnect_capture(&error)) return true;

  // Fallback: close + reopen capture stream.
  RCLCPP_INFO(get_logger(), "Capture reconnect via close/reopen");
  capture_backend_->close();
  if (!capture_backend_->open_capture(capture_config_, &error)) {
    RCLCPP_WARN(get_logger(), "Capture reopen failed: %s", error.c_str());
    return false;
  }
  if (!capture_backend_->open_playback(capture_config_, &error)) {
    capture_backend_->close();
    RCLCPP_WARN(get_logger(), "Playback reopen after capture reconnect failed: %s", error.c_str());
    return false;
  }
  return true;
}

}  // namespace k1muse_voice_audio
