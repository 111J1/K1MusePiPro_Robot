#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "k1muse_ai_runtime/audio/audio_frame_queue.hpp"
#include "k1muse_ai_runtime/audio/audio_frame_validator.hpp"
#include "k1muse_ai_runtime/bounded_cpu_worker.hpp"
#include "k1muse_ai_runtime/control_snapshot.hpp"
#include "k1muse_ai_runtime/model_runtime.hpp"
#include "k1muse_ai_runtime/models/vision_runtime.hpp"
#include "k1muse_ai_runtime/resource_guard.hpp"
#include "k1muse_ai_runtime/runtime_config.hpp"
#include "k1muse_ai_runtime/runtime_core.hpp"
#include "k1muse_ai_runtime/runtime_scheduler.hpp"
#include "k1muse_ai_runtime_msgs/msg/ai_runtime_control.hpp"
#include "k1muse_ai_runtime_msgs/msg/ai_runtime_state.hpp"
#include "k1muse_audio_msgs/msg/audio_frame.hpp"
#include "k1muse_vision_msgs/msg/detection2_d_frame.hpp"
#include "k1muse_voice_msgs/msg/listen_event.hpp"
#include "k1muse_voice_msgs/msg/listen_result.hpp"
#include "k1muse_voice_msgs/msg/tts_play_request.hpp"
#include "k1muse_voice_msgs/msg/tts_status.hpp"
#include "k1muse_voice_msgs/msg/tts_text_request.hpp"
#include "k1muse_voice_msgs/msg/wakeword_event.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace k1muse_ai_runtime
{

class AiRuntimeNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using ControlMessage = k1muse_ai_runtime_msgs::msg::AiRuntimeControl;
  using StateMessage = k1muse_ai_runtime_msgs::msg::AiRuntimeState;
  using AudioFrameMessage = k1muse_audio_msgs::msg::AudioFrame;
  using WakewordEventMessage = k1muse_voice_msgs::msg::WakewordEvent;
  using ListenEventMessage = k1muse_voice_msgs::msg::ListenEvent;
  using ListenResultMessage = k1muse_voice_msgs::msg::ListenResult;
  using ImageMessage = sensor_msgs::msg::Image;
  using Detection2DFrameMessage = k1muse_vision_msgs::msg::Detection2DFrame;
  using Detection2DMessage = k1muse_vision_msgs::msg::Detection2D;
  using TtsTextRequestMessage = k1muse_voice_msgs::msg::TtsTextRequest;
  using TtsStatusMessage = k1muse_voice_msgs::msg::TtsStatus;
  using TtsPlayRequestMessage = k1muse_voice_msgs::msg::TtsPlayRequest;

  explicit AiRuntimeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~AiRuntimeNode() override;

  bool runtime_ready() const noexcept;
  std::string last_error() const;
  ControlSnapshot control_snapshot() const;
  StateMessage last_published_state() const;
  std::size_t retained_model_count() const;
  std::size_t cpu_worker_count() const;
  QueueStats scheduler_stats() const;
  bool update_control_snapshot(const ControlMessage & message);

protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  struct AudioFrameData
  {
    std::vector<int16_t> pcm;
    size_t samples{0};
    std::string trace_id;
    uint64_t epoch{0};
    uint64_t seq{0};
  };

  void on_control(const ControlMessage::SharedPtr message);
  void on_audio_frame(const AudioFrameMessage::SharedPtr message);
  void on_image_frame(const ImageMessage::SharedPtr message);
  void on_tts_text_request(const TtsTextRequestMessage::SharedPtr message);
  void reset_current_turn();
  void publish_state(
    bool ready, const std::string & error = {},
    std::optional<uint8_t> lifecycle_state = std::nullopt,
    const std::string & lifecycle_name = {});
  bool warmup_models();
  bool warmup_model(ModelRuntime & model, JobModule module);
  void start_workers();
  bool stop_workers();
  bool release_resources();
  void set_last_error(const std::string & error);
  bool epoch_is_current(uint64_t epoch) const;
  bool module_enabled(JobModule module) const;
  bool try_commit(
    uint64_t epoch, JobModule module,
    std::chrono::steady_clock::time_point deadline,
    const std::function<bool()> & cancel_predicate,
    const CancellationToken & token,
    const std::function<void()> & commit);
  void on_worker_state_changed();
  BoundedCpuWorker * cpu_worker(JobModule module);

  RuntimeConfig config_;
  std::vector<ModelRuntime *> models_;

  rclcpp::CallbackGroup::SharedPtr control_callback_group_;
  rclcpp::CallbackGroup::SharedPtr state_callback_group_;
  rclcpp::CallbackGroup::SharedPtr model_data_callback_group_;
  rclcpp::Publisher<StateMessage>::SharedPtr state_publisher_;
  rclcpp::Publisher<WakewordEventMessage>::SharedPtr wakeword_event_publisher_;
  rclcpp::Publisher<ListenEventMessage>::SharedPtr listen_event_publisher_;
  rclcpp::Publisher<ListenResultMessage>::SharedPtr listen_result_publisher_;
  rclcpp::Subscription<ControlMessage>::SharedPtr control_subscription_;
  rclcpp::Subscription<AudioFrameMessage>::SharedPtr audio_subscription_;
  rclcpp::Subscription<ImageMessage>::SharedPtr image_subscription_;
  rclcpp::Subscription<TtsTextRequestMessage>::SharedPtr tts_text_subscription_;
  rclcpp::Publisher<Detection2DFrameMessage>::SharedPtr detection_2d_publisher_;
  rclcpp::Publisher<TtsStatusMessage>::SharedPtr tts_status_publisher_;
  rclcpp::Publisher<TtsPlayRequestMessage>::SharedPtr tts_play_publisher_;

  std::unique_ptr<AudioFrameValidator> audio_validator_;
  AudioFrameQueue<AudioFrameData> wakeword_queue_{60};
  AudioFrameQueue<AudioFrameData> vad_asr_queue_{60};

  // Owned T2 runtimes; models_ holds raw pointers for polymorphic iteration.
  std::unique_ptr<class WakewordRuntime> wakeword_runtime_;
  std::unique_ptr<class VadAsrRuntime> vad_asr_runtime_;
  // T3: owned VisionRuntime (V1 path, retained for compatibility)
  std::unique_ptr<VisionRuntime> vision_runtime_;
  // V2: multi-detector vision pipeline + profile manager + voice guard
  std::unique_ptr<RuntimeCore> runtime_core_;
  // T4: owned TTSRuntime
  std::unique_ptr<class TTSRuntime> tts_runtime_;
  std::vector<std::unique_ptr<ModelRuntime>> mock_models_;

  mutable std::mutex control_mutex_;
  mutable std::mutex control_update_mutex_;
  ControlSnapshot control_snapshot_;
  mutable std::mutex state_mutex_;
  StateMessage last_published_state_;
  mutable std::mutex error_mutex_;
  std::string last_error_;
  std::atomic<bool> runtime_ready_{false};
  std::atomic<bool> worker_state_updates_enabled_{true};

  // Declared last so reverse destruction joins workers before model/state ownership is released.
  CommitGenerationGate commit_generation_gate_;
  std::unique_ptr<ResourceGuard> guard_;
  std::unique_ptr<RuntimeScheduler> scheduler_;
  std::map<JobModule, std::unique_ptr<BoundedCpuWorker>> cpu_workers_;
};

}  // namespace k1muse_ai_runtime
