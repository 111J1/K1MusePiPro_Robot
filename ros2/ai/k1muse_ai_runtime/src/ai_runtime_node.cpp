#include "k1muse_ai_runtime/ai_runtime_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <future>
#include <stdexcept>
#include <thread>
#include <utility>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/create_publisher.hpp>

#include "k1muse_ai_runtime/backend_factory.hpp"
#include "k1muse_ai_runtime/models/tts_runtime.hpp"
#include "k1muse_ai_runtime/models/vad_asr_runtime.hpp"
#include "k1muse_ai_runtime/models/vision_runtime.hpp"
#include "k1muse_ai_runtime/models/wakeword_runtime.hpp"
#include "k1muse_common/qos_profiles.hpp"

namespace k1muse_ai_runtime
{
namespace
{

class MockModelRuntime final : public ModelRuntime
{
public:
  MockModelRuntime(
    std::string name, std::string provider, int warmup_delay_ms,
    bool fail_warmup)
  : name_(std::move(name)),
    provider_(std::move(provider)),
    warmup_delay_ms_(warmup_delay_ms),
    fail_warmup_(fail_warmup)
  {
  }

  const std::string & name() const override {return name_;}
  const std::string & provider() const override {return provider_;}
  void load(const CancellationToken & token, Deadline deadline) override
  {
    if (token.stop_requested() || std::chrono::steady_clock::now() > deadline) {
      throw std::runtime_error(name_ + " load cancelled or expired");
    }
    cancel_requested_.store(false);
    loaded_ = true;
  }

  void warmup(const CancellationToken & token, Deadline deadline) override
  {
    if (!loaded_) {
      throw std::runtime_error(name_ + " is not loaded");
    }
    const auto warmup_end =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(warmup_delay_ms_);
    while (std::chrono::steady_clock::now() < warmup_end) {
      if (token.stop_requested() || cancel_requested_.load() ||
        std::chrono::steady_clock::now() > deadline)
      {
        throw std::runtime_error(name_ + " warmup cancelled");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (fail_warmup_) {
      throw std::runtime_error(name_ + " mock warmup failure");
    }
  }

  void request_cancel() noexcept override {cancel_requested_.store(true);}
  bool stop(std::chrono::milliseconds) noexcept override {return true;}
  void final_join() noexcept override {}
  void unload() noexcept override {loaded_ = false;}
  bool loaded() const noexcept override {return loaded_;}

private:
  std::string name_;
  std::string provider_;
  int warmup_delay_ms_;
  bool fail_warmup_;
  bool loaded_{false};
  std::atomic<bool> cancel_requested_{false};
};

const std::vector<JobModule> kCpuStartOrder{
  JobModule::WAKEWORD_CPU,
  JobModule::VAD_CPU,
  JobModule::ASR_CPU,
  JobModule::VISION_CPU,
  JobModule::TTS_CPU,
};

}  // namespace

AiRuntimeNode::AiRuntimeNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("ai_runtime", options)
{
  state_callback_group_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::PublisherOptions publisher_options;
  publisher_options.callback_group = state_callback_group_;
  state_publisher_ = rclcpp::create_publisher<StateMessage>(
    *this, "/ai_runtime/state",
    k1muse_common::qos::LatchedState(), publisher_options);
  wakeword_event_publisher_ = rclcpp::create_publisher<WakewordEventMessage>(
    *this, "/ai_runtime/wakeword/event",
    k1muse_common::qos::ReliableEvent(5), publisher_options);
  listen_event_publisher_ = rclcpp::create_publisher<ListenEventMessage>(
    *this, "/ai_runtime/listen/event",
    k1muse_common::qos::ReliableEvent(10), publisher_options);
  listen_result_publisher_ = rclcpp::create_publisher<ListenResultMessage>(
    *this, "/voice/listen/result",
    k1muse_common::qos::ReliableResult(10), publisher_options);
  tts_status_publisher_ = rclcpp::create_publisher<TtsStatusMessage>(
    *this, "/voice/tts/status",
    k1muse_common::qos::LatchedState(1), publisher_options);
  tts_play_publisher_ = rclcpp::create_publisher<TtsPlayRequestMessage>(
    *this, "/voice/tts/play",
    k1muse_common::qos::ReliableEvent(5), publisher_options);
}

AiRuntimeNode::~AiRuntimeNode()
{
  if (scheduler_) {
    scheduler_->set_state_callback({});
  }
  for (auto & entry : cpu_workers_) {
    entry.second->set_state_callback({});
  }
  if (!release_resources()) {
    wakeword_queue_.clear();
    vad_asr_queue_.clear();
    for (auto & model : models_) {
      model->request_cancel();
    }
    cpu_workers_.clear();
    scheduler_.reset();
    for (auto & model : models_) {
      model->final_join();
      model->unload();
    }
    models_.clear();
    mock_models_.clear();
    vision_runtime_.reset();
    tts_runtime_.reset();
    wakeword_runtime_.reset();
    vad_asr_runtime_.reset();
    audio_validator_.reset();
    audio_subscription_.reset();
    image_subscription_.reset();
    detection_2d_publisher_.reset();
    guard_.reset();
  }
}

bool AiRuntimeNode::runtime_ready() const noexcept
{
  return runtime_ready_.load();
}

std::string AiRuntimeNode::last_error() const
{
  std::lock_guard<std::mutex> lock(error_mutex_);
  return last_error_;
}

ControlSnapshot AiRuntimeNode::control_snapshot() const
{
  std::lock_guard<std::mutex> lock(control_mutex_);
  return control_snapshot_;
}

AiRuntimeNode::StateMessage AiRuntimeNode::last_published_state() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return last_published_state_;
}

std::size_t AiRuntimeNode::retained_model_count() const
{
  return models_.size();
}

std::size_t AiRuntimeNode::cpu_worker_count() const
{
  return cpu_workers_.size();
}

QueueStats AiRuntimeNode::scheduler_stats() const
{
  return scheduler_ ? scheduler_->stats() : QueueStats{};
}

bool AiRuntimeNode::update_control_snapshot(const ControlMessage & message)
{
  std::lock_guard<std::mutex> update_lock(control_update_mutex_);
  if (!commit_generation_gate_.update(
      message.epoch, message.wakeword_enabled, message.vad_asr_enabled,
      message.vision_enabled, message.tts_enabled))
  {
    set_last_error("stale control epoch " + std::to_string(message.epoch));
    RCLCPP_WARN(get_logger(), "%s", last_error().c_str());
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(control_mutex_);
    // Same-epoch messages replace the full snapshot; supervisor is the sole owner.
    control_snapshot_.trace_id = message.trace_id;
    control_snapshot_.epoch = message.epoch;
    control_snapshot_.interaction_state = message.interaction_state;
    control_snapshot_.interaction_state_name = message.interaction_state_name;
    control_snapshot_.wakeword_enabled = message.wakeword_enabled;
    control_snapshot_.vision_enabled = message.vision_enabled;
    control_snapshot_.vad_asr_enabled = message.vad_asr_enabled;
    control_snapshot_.tts_enabled = message.tts_enabled;
    control_snapshot_.reason = message.reason;
  }
  publish_state(runtime_ready());
  return true;
}

AiRuntimeNode::CallbackReturn AiRuntimeNode::on_configure(
  const rclcpp_lifecycle::State &)
{
  try {
    config_ = RuntimeConfig::declare_and_load(*this);
    const auto validation_error = config_.validate();
    if (!validation_error.empty()) {
      set_last_error(validation_error);
      RCLCPP_ERROR(get_logger(), "T1 configuration rejected: %s", validation_error.c_str());
      publish_state(
        false, validation_error,
        lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
      return CallbackReturn::FAILURE;
    }

    // Set SpacemiT EP environment variables before SDK initialization.
    setenv("SPACEMIT_EP_USE_GLOBAL_INTRA_THREAD", "1", 1);
    setenv("SPACEMIT_EP_INTRA_THREAD_NUM",
      std::to_string(config_.spacemit_ep_intra_threads).c_str(), 1);
    setenv("SPACEMIT_EP_INTER_THREAD_NUM",
      std::to_string(config_.spacemit_ep_inter_threads).c_str(), 1);

    guard_ = std::make_unique<ResourceGuard>();
    scheduler_ = std::make_unique<RuntimeScheduler>(
      *guard_, static_cast<std::size_t>(config_.guarded_queue_capacity),
      std::chrono::milliseconds(config_.stop_timeout_ms));
    scheduler_->set_state_callback([this]() {on_worker_state_changed();});

    cpu_workers_.clear();
    for (const auto module : kCpuStartOrder) {
      auto worker = std::make_unique<BoundedCpuWorker>(
        module, static_cast<std::size_t>(config_.cpu_queue_capacity),
        std::chrono::milliseconds(config_.stop_timeout_ms));
      worker->set_state_callback([this]() {on_worker_state_changed();});
      cpu_workers_.emplace(module, std::move(worker));
    }

    control_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    model_data_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = control_callback_group_;
    control_subscription_ = create_subscription<ControlMessage>(
      "/ai_runtime/control", k1muse_common::qos::LatchedState(),
      std::bind(&AiRuntimeNode::on_control, this, std::placeholders::_1),
      subscription_options);

    rclcpp::SubscriptionOptions audio_subscription_options;
    audio_subscription_options.callback_group = model_data_callback_group_;
    audio_subscription_ = create_subscription<AudioFrameMessage>(
      "/audio/raw_pcm", k1muse_common::qos::AudioStream(20),
      std::bind(&AiRuntimeNode::on_audio_frame, this, std::placeholders::_1),
      audio_subscription_options);

    // T2: create wakeword runtime via factory
    auto wakeword_backend = create_wakeword_backend(config_);
    wakeword_runtime_ = std::make_unique<WakewordRuntime>(
      std::move(wakeword_backend),
      [this](const std::string & trace_id, uint64_t epoch,
             float confidence, const std::string & keyword) {
        if (confidence < config_.wakeword_threshold) {
          return;
        }
        WakewordEventMessage msg;
        msg.header.stamp = now();
        msg.trace_id = trace_id;
        msg.epoch = epoch;
        msg.event = WakewordEventMessage::EVENT_DETECTED;
        msg.keyword = keyword;
        msg.confidence = confidence;
        wakeword_event_publisher_->publish(msg);
      });

    // T2: create VAD+ASR runtime via factory
    auto vad_backend = create_vad_backend(config_);
    auto asr_backend = create_asr_backend(config_);
    VadSegmenter::Config seg_config;
    seg_config.sample_rate = 16000;
    seg_config.audio_chunk_ms = 20;
    seg_config.vad_threshold = config_.vad_threshold;
    seg_config.min_speech_ms = static_cast<uint32_t>(config_.min_speech_ms);
    seg_config.end_silence_ms = static_cast<uint32_t>(config_.end_silence_ms);
    seg_config.pre_roll_ms = static_cast<uint32_t>(config_.pre_roll_ms);
    seg_config.post_pad_ms = static_cast<uint32_t>(config_.post_pad_ms);
    seg_config.max_utterance_ms = static_cast<uint32_t>(config_.max_utterance_ms);
    vad_asr_runtime_ = std::make_unique<VadAsrRuntime>(
      std::move(vad_backend), std::move(asr_backend), seg_config,
      config_.asr_provider,
      [this](const std::string & trace_id, const std::string & utterance_id,
             uint64_t epoch, VadAsrRuntime::ListenEvent event,
             const std::string & reason) {
        ListenEventMessage msg;
        msg.header.stamp = now();
        msg.trace_id = trace_id;
        msg.utterance_id = utterance_id;
        msg.epoch = epoch;
        switch (event) {
          case VadAsrRuntime::ListenEvent::SPEECH_START:
            msg.event = ListenEventMessage::EVENT_SPEECH_START;
            msg.event_name = "speech_start";
            break;
          case VadAsrRuntime::ListenEvent::SPEECH_END:
            msg.event = ListenEventMessage::EVENT_SPEECH_END;
            msg.event_name = "speech_end";
            break;
          case VadAsrRuntime::ListenEvent::ASR_STARTED:
            msg.event = ListenEventMessage::EVENT_ASR_STARTED;
            msg.event_name = "asr_started";
            break;
          case VadAsrRuntime::ListenEvent::ASR_DONE:
            msg.event = ListenEventMessage::EVENT_ASR_DONE;
            msg.event_name = "asr_done";
            break;
          case VadAsrRuntime::ListenEvent::FAILED:
            msg.event = ListenEventMessage::EVENT_FAILED;
            msg.event_name = "failed";
            break;
          case VadAsrRuntime::ListenEvent::CANCELLED:
            msg.event = ListenEventMessage::EVENT_CANCELLED;
            msg.event_name = "cancelled";
            break;
        }
        msg.reason = reason;
        listen_event_publisher_->publish(msg);
      },
      [this](const std::string & trace_id, const std::string & utterance_id,
             uint64_t epoch, bool success, const std::string & text,
             float confidence, const std::string & language,
             const std::string & reason) {
        ListenResultMessage msg;
        msg.header.stamp = now();
        msg.trace_id = trace_id;
        msg.utterance_id = utterance_id;
        msg.epoch = epoch;
        msg.success = success;
        msg.text = text;
        msg.confidence = confidence;
        msg.language = language;
        msg.reason = reason;
        listen_result_publisher_->publish(msg);
      },
      // SegmentReady callback: build ASR InferenceJob and submit to scheduler or cpu_worker.
      [this](VadAsrRuntime::SegmentData segment) {
        const auto asr_module = (config_.asr_provider == "ep")
          ? JobModule::ASR_EP : JobModule::ASR_CPU;

        auto seg = std::make_shared<VadAsrRuntime::SegmentData>(std::move(segment));
        InferenceJob asr_job;
        asr_job.id = "asr:" + seg->utterance_id;
        asr_job.module = asr_module;
        asr_job.provider_class = canonical_provider(asr_module);
        asr_job.priority = canonical_priority(asr_module);
        asr_job.epoch = seg->epoch;
        asr_job.deadline = std::chrono::steady_clock::now()
          + std::chrono::milliseconds(config_.job_deadline_ms);
        asr_job.epoch_is_current = [this](uint64_t e) {return epoch_is_current(e);};
        asr_job.module_enabled = [this](JobModule m) {return module_enabled(m);};
        asr_job.cancel_predicate = [this]() {return !runtime_ready();};

        auto * runtime_ptr = vad_asr_runtime_.get();
        asr_job.execute = [runtime_ptr, seg](const CancellationToken & token) {
          if (!token.stop_requested()) {
            runtime_ptr->run_asr(*seg);
          }
        };
        asr_job.commit_if_current =
          [this, epoch = seg->epoch, asr_module](
            const CancellationToken & token) {
            return try_commit(
              epoch, asr_module,
              std::chrono::steady_clock::now() + std::chrono::milliseconds(config_.job_deadline_ms),
              [this]() {return !runtime_ready();}, token, [](){});
          };

        if (canonical_provider(asr_module) == ProviderClass::GuardedEp) {
          scheduler_->submit(std::move(asr_job));
        } else {
          cpu_worker(asr_module)->submit(std::move(asr_job));
        }
      });

    // T2: audio frame validator
    audio_validator_ = std::make_unique<AudioFrameValidator>();

    // T3: create VisionRuntime via factory
    auto vision_backend = create_vision_backend(config_);
    vision_runtime_ = std::make_unique<VisionRuntime>(
      std::move(vision_backend),
      config_.vision_provider,
      [this](const std::string & trace_id, uint64_t epoch,
             uint32_t image_width, uint32_t image_height,
             std::vector<VisionBackend::Detection> detections) {
        Detection2DFrameMessage msg;
        msg.header.stamp = now();
        msg.trace_id = trace_id;
        msg.epoch = epoch;
        msg.image_width = image_width;
        msg.image_height = image_height;
        for (const auto & det : detections) {
          Detection2DMessage d;
          d.detection_id = det.detection_id;
          d.class_name = det.class_name;
          d.score = det.score;
          d.x = det.x;
          d.y = det.y;
          d.width = det.width;
          d.height = det.height;
          msg.detections.push_back(d);
        }
        detection_2d_publisher_->publish(msg);
      });

    // T3: image subscription with SensorLatest(3) QoS
    rclcpp::SubscriptionOptions image_subscription_options;
    image_subscription_options.callback_group = model_data_callback_group_;
    image_subscription_ = create_subscription<ImageMessage>(
      "/camera/main/color/image_raw", k1muse_common::qos::SensorLatest(3),
      std::bind(&AiRuntimeNode::on_image_frame, this, std::placeholders::_1),
      image_subscription_options);

    // T3: detection publisher with ReliableResult(5) QoS
    rclcpp::PublisherOptions detection_publisher_options;
    detection_publisher_options.callback_group = state_callback_group_;
    detection_2d_publisher_ = rclcpp::create_publisher<Detection2DFrameMessage>(
      *this, "/vision/detection_2d",
      k1muse_common::qos::ReliableResult(5), detection_publisher_options);

    // ── V2 multi-detector pipeline ──
    RCLCPP_INFO(
      get_logger(),
      "[startup] ai_runtime backend_mode=%s provider_fallback=%s "
      "providers={wakeword:%s vad:%s asr:%s vision:%s tts:%s} "
      "queues={guarded:%d cpu:%d} deadlines={job_ms:%d stop_ms:%d} "
      "topics={control_in:/ai_runtime/control audio_in:/audio/raw_pcm "
      "camera_in:/camera/main/color/image_raw detection_out:/vision/detection_2d "
      "tts_in:/voice/tts/text state:/ai_runtime/state}",
      config_.backend_mode.c_str(), config_.provider_fallback.c_str(),
      config_.wakeword_provider.c_str(), config_.vad_provider.c_str(),
      config_.asr_provider.c_str(), config_.vision_provider.c_str(),
      config_.tts_provider.c_str(), config_.guarded_queue_capacity,
      config_.cpu_queue_capacity, config_.job_deadline_ms,
      config_.stop_timeout_ms);
    RCLCPP_INFO(
      get_logger(),
      "[startup] ai_runtime models={kws:%s kws_keywords:%s vad:%s asr:%s "
      "vision:%s vision_config:%s vision_labels:%s tts:%s} "
      "spacemit_ep={intra:%d inter:%d}",
      config_.sherpa_kws_model_path.c_str(),
      config_.sherpa_kws_keywords_path.c_str(),
      config_.vad_model_path.c_str(), config_.sensevoice_asr_model_path.c_str(),
      config_.vision_model_path.c_str(), config_.vision_config_path.c_str(),
      config_.vision_labels_path.c_str(), config_.tts_model_path.c_str(),
      config_.spacemit_ep_intra_threads, config_.spacemit_ep_inter_threads);
    runtime_core_ = std::make_unique<RuntimeCore>();
    // V2 detection callback: publish Detection2DFrame via existing publisher.
    runtime_core_->vision().set_detection_callback(
        [this](const VisionPipeline::Detection2DOutput& out) {
          Detection2DFrameMessage msg;
          msg.header.stamp = now();
          msg.header.frame_id = out.frame_id;
          msg.trace_id = "";
          for (const auto& d : out.detections) {
            Detection2DMessage det;
            det.detection_id = d.detection_id;
            det.class_name = d.class_name;
            det.score = d.score;
            det.x = d.x;
            det.y = d.y;
            det.width = d.width;
            det.height = d.height;
            msg.detections.push_back(det);
          }
          detection_2d_publisher_->publish(msg);
        });
    // TODO(V2): add AlertRosPublisher + Pose2DFrame publisher after
    // k1muse_ai_runtime_msgs is rebuilt with updated AiruntimeAlert/Pose2D/Pose2DFrame.

    // T4: create TTSRuntime via factory
    auto tts_backend = create_tts_backend(config_);
    tts_runtime_ = std::make_unique<TTSRuntime>(
      std::move(tts_backend),
      config_.tts_provider,
      TTSRuntime::StatusCallback{
        [this](const std::string & trace_id, const std::string & request_id,
               uint64_t epoch, const std::string & source,
               uint8_t state, const std::string & state_name,
               const std::string & reason) {
          TtsStatusMessage msg;
          msg.header.stamp = now();
          msg.trace_id = trace_id;
          msg.request_id = request_id;
          msg.epoch = epoch;
          msg.source = source;
          msg.state = state;
          msg.state_name = state_name;
          msg.reason = reason;
          tts_status_publisher_->publish(msg);
        },
        [this](const std::string & trace_id, const std::string & request_id,
               uint64_t epoch, const std::string & source,
               const TTSRuntime::PcmResult & pcm) {
          TtsPlayRequestMessage msg;
          msg.header.stamp = now();
          msg.trace_id = trace_id;
          msg.request_id = request_id;
          msg.epoch = epoch;
          msg.source = source;
          msg.sample_rate = pcm.sample_rate;
          msg.channels = pcm.channels;
          msg.encoding = pcm.encoding;
          msg.pcm_s16le = pcm.pcm_s16le;
          tts_play_publisher_->publish(msg);
        }
      });

    // T4: TTS text subscription with ReliableEvent(5) QoS
    rclcpp::SubscriptionOptions tts_subscription_options;
    tts_subscription_options.callback_group = model_data_callback_group_;
    tts_text_subscription_ = create_subscription<TtsTextRequestMessage>(
      "/voice/tts/text", k1muse_common::qos::ReliableEvent(5),
      std::bind(&AiRuntimeNode::on_tts_text_request, this, std::placeholders::_1),
      tts_subscription_options);

    // Build models vector (raw pointers for polymorphic lifecycle iteration).
    mock_models_.clear();

    models_.clear();
    models_.push_back(wakeword_runtime_.get());
    models_.push_back(vad_asr_runtime_.get());
    models_.push_back(vision_runtime_.get());
    models_.push_back(tts_runtime_.get());

    CancellationToken load_token;
    const auto load_deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(config_.job_deadline_ms);
    for (auto & model : models_) {
      model->load(load_token, load_deadline);
    }
    runtime_ready_.store(false);
    worker_state_updates_enabled_.store(true);
    commit_generation_gate_.update(
      control_snapshot().epoch, true, true, true, true);
    set_last_error({});
    publish_state(
      false, {}, lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
    return CallbackReturn::SUCCESS;
  } catch (const std::exception & error) {
    set_last_error(error.what());
    RCLCPP_ERROR(get_logger(), "configure failed: %s", error.what());
    publish_state(
      false, error.what(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
    release_resources();
    return CallbackReturn::ERROR;
  }
}

AiRuntimeNode::CallbackReturn AiRuntimeNode::on_activate(
  const rclcpp_lifecycle::State &)
{
  worker_state_updates_enabled_.store(true);
  scheduler_->start();
  start_workers();
  if (!warmup_models()) {
    runtime_ready_.store(false);
    publish_state(false, last_error());
    return CallbackReturn::ERROR;
  }

  // Long-running jobs use runtime_ready() in both their admission predicate and
  // execution loop. Mark the runtime ready before submitting them; otherwise a
  // fast worker can reject the consumer before this callback reaches its end.
  wakeword_queue_.restart();
  vad_asr_queue_.restart();
  const auto control = control_snapshot();
  commit_generation_gate_.update(
    control.epoch, control.wakeword_enabled, control.vad_asr_enabled,
    control.vision_enabled, control.tts_enabled);
  runtime_ready_.store(true);

  const auto fail_consumer_start = [this](const std::string & error) {
      runtime_ready_.store(false);
      set_last_error(error);
      wakeword_queue_.stop();
      vad_asr_queue_.stop();
      publish_state(false, error);
      return CallbackReturn::ERROR;
    };

  // T2: submit queue-draining jobs for wakeword and VAD/ASR audio processing.
  {
    auto wakeword_runtime_ptr = wakeword_runtime_.get();
    InferenceJob ww_job;
    ww_job.id = "audio:wakeword";
    ww_job.module = JobModule::WAKEWORD_CPU;
    ww_job.provider_class = canonical_provider(JobModule::WAKEWORD_CPU);
    ww_job.priority = canonical_priority(JobModule::WAKEWORD_CPU);
    ww_job.epoch = control_snapshot().epoch;
    ww_job.deadline =
      std::chrono::steady_clock::now() + std::chrono::hours(24);
    ww_job.epoch_is_current = [this](uint64_t) {return runtime_ready();};
    ww_job.module_enabled = [this](JobModule) {return runtime_ready();};
    ww_job.cancel_predicate = [this]() {return !runtime_ready();};
    ww_job.execute =
      [this, wakeword_runtime_ptr](const CancellationToken & token) {
        while (!token.stop_requested() && runtime_ready()) {
          auto frame = wakeword_queue_.wait_pop();
          if (frame) {
            wakeword_runtime_ptr->process_audio(
              frame->pcm.data(), frame->samples,
              frame->trace_id, frame->epoch, control_snapshot());
          }
        }
      };
    ww_job.commit_if_current =
      [this, epoch = ww_job.epoch](const CancellationToken & token) {
        return try_commit(
          epoch, JobModule::WAKEWORD_CPU,
          std::chrono::steady_clock::now() + std::chrono::hours(24),
          [this]() {return !runtime_ready();}, token, [](){});
      };
    if (!cpu_worker(JobModule::WAKEWORD_CPU)->submit(std::move(ww_job))) {
      return fail_consumer_start("failed to start wakeword audio consumer");
    }
  }

  {
    auto vad_asr_runtime_ptr = vad_asr_runtime_.get();
    InferenceJob vad_job;
    vad_job.id = "audio:vad_asr";
    vad_job.module = JobModule::VAD_CPU;
    vad_job.provider_class = canonical_provider(JobModule::VAD_CPU);
    vad_job.priority = canonical_priority(JobModule::VAD_CPU);
    vad_job.epoch = control_snapshot().epoch;
    vad_job.deadline =
      std::chrono::steady_clock::now() + std::chrono::hours(24);
    vad_job.epoch_is_current = [this](uint64_t) {return runtime_ready();};
    vad_job.module_enabled = [this](JobModule) {return runtime_ready();};
    vad_job.cancel_predicate = [this]() {return !runtime_ready();};
    vad_job.execute =
      [this, vad_asr_runtime_ptr](const CancellationToken & token) {
        while (!token.stop_requested() && runtime_ready()) {
          auto frame = vad_asr_queue_.wait_pop();
          if (frame) {
            vad_asr_runtime_ptr->process_vad_only(
              frame->pcm.data(), frame->samples,
              frame->trace_id, frame->epoch, control_snapshot(),
              frame->seq);
          }
        }
      };
    vad_job.commit_if_current =
      [this, epoch = vad_job.epoch](const CancellationToken & token) {
        return try_commit(
          epoch, JobModule::VAD_CPU,
          std::chrono::steady_clock::now() + std::chrono::hours(24),
          [this]() {return !runtime_ready();}, token, [](){});
      };
    if (!cpu_worker(JobModule::VAD_CPU)->submit(std::move(vad_job))) {
      return fail_consumer_start("failed to start VAD audio consumer");
    }
  }

  // T3: vision inference — V2 discrete-job driver or V1 fallback.
  if (runtime_core_) {
    // ── V2 driver: runs on VISION_CPU, submits discrete EP jobs ──
    // Each job does ONE inference → completes → scheduler freed for
    // higher-priority ASR_EP between vision jobs.
    auto vision_runtime_ptr = vision_runtime_.get();
    InferenceJob vision_driver;
    vision_driver.id = "vision:v2_driver";
    vision_driver.module = JobModule::VISION_CPU;
    vision_driver.provider_class = ProviderClass::Cpu;
    vision_driver.priority = canonical_priority(JobModule::VISION_CPU);
    vision_driver.epoch = control_snapshot().epoch;
    vision_driver.deadline =
        std::chrono::steady_clock::now() + std::chrono::hours(24);
    vision_driver.epoch_is_current = [this](uint64_t) {
      return runtime_ready();
    };
    vision_driver.module_enabled = [this](JobModule) {
      return runtime_ready();
    };
    vision_driver.cancel_predicate = [this]() {
      return !runtime_ready();
    };
    vision_driver.execute =
        [this, vision_runtime_ptr](const CancellationToken& token) {
          while (!token.stop_requested() && runtime_ready()) {
            // V2 path: build discrete job via VisionPipeline.
            auto v2_job = runtime_core_->vision().build_next_job(
                control_snapshot().epoch,
                [this](uint64_t e) { return epoch_is_current(e); },
                [this](JobModule m) { return module_enabled(m); },
                [this]() { return !runtime_ready(); },
                std::chrono::steady_clock::now());
            if (v2_job) {
              scheduler_->submit(std::move(*v2_job));
            }
            // V1 fallback: keep old VisionRuntime running for
            // backward compatibility until V2 fully takes over.
            bool processed = vision_runtime_ptr->process_latest_frame(
                [this](uint64_t e) { return epoch_is_current(e); },
                [this](JobModule m) { return module_enabled(m); },
                [this]() { return !runtime_ready(); },
                std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(100),
                control_snapshot().epoch);
            if (!processed && !v2_job) {
              std::this_thread::sleep_for(
                  std::chrono::milliseconds(10));
            }
          }
        };
    vision_driver.commit_if_current =
        [this, epoch = vision_driver.epoch](
            const CancellationToken& token) {
          return try_commit(
              epoch, JobModule::VISION_CPU,
              std::chrono::steady_clock::now() + std::chrono::hours(24),
              [this]() { return !runtime_ready(); }, token, []() {});
        };
    cpu_worker(JobModule::VISION_CPU)->submit(std::move(vision_driver));
  } else {
    // ── V1 fallback: single long-running EP job (legacy) ──
    auto vision_runtime_ptr = vision_runtime_.get();
    const auto vision_module = config_.vision_provider == "ep"
                                   ? JobModule::VISION_EP
                                   : JobModule::VISION_CPU;
    InferenceJob vision_job;
    vision_job.id = "vision:inference";
    vision_job.module = vision_module;
    vision_job.provider_class = canonical_provider(vision_module);
    vision_job.priority = canonical_priority(vision_module);
    vision_job.epoch = control_snapshot().epoch;
    vision_job.deadline =
        std::chrono::steady_clock::now() + std::chrono::hours(24);
    vision_job.epoch_is_current = [this](uint64_t) {
      return runtime_ready();
    };
    vision_job.module_enabled = [this](JobModule) {
      return runtime_ready();
    };
    vision_job.cancel_predicate = [this]() {
      return !runtime_ready();
    };
    vision_job.execute =
        [this, vision_runtime_ptr](const CancellationToken& token) {
          while (!token.stop_requested() && runtime_ready()) {
            bool processed = vision_runtime_ptr->process_latest_frame(
                [this](uint64_t e) { return epoch_is_current(e); },
                [this](JobModule m) { return module_enabled(m); },
                [this]() { return !runtime_ready(); },
                std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(100),
                control_snapshot().epoch);
            if (!processed) {
              std::this_thread::sleep_for(
                  std::chrono::milliseconds(10));
            }
          }
        };
    vision_job.commit_if_current =
        [this, epoch = vision_job.epoch, vision_module](
            const CancellationToken& token) {
          return try_commit(
              epoch, vision_module,
              std::chrono::steady_clock::now() + std::chrono::hours(24),
              [this]() { return !runtime_ready(); }, token, []() {});
        };
    if (canonical_provider(vision_module) == ProviderClass::GuardedEp) {
      scheduler_->submit(std::move(vision_job));
    } else {
      cpu_worker(vision_module)->submit(std::move(vision_job));
    }
  }

  // T4: submit long-running TTS inference job.
  {
    auto tts_runtime_ptr = tts_runtime_.get();
    const auto tts_module =
      config_.tts_provider == "ep" ? JobModule::TTS_EP : JobModule::TTS_CPU;
    InferenceJob tts_job;
    tts_job.id = "tts:inference";
    tts_job.module = tts_module;
    tts_job.provider_class = canonical_provider(tts_module);
    tts_job.priority = canonical_priority(tts_module);
    tts_job.epoch = control_snapshot().epoch;
    tts_job.deadline =
      std::chrono::steady_clock::now() + std::chrono::hours(24);
    tts_job.epoch_is_current = [this](uint64_t) {return runtime_ready();};
    tts_job.module_enabled = [this](JobModule) {return runtime_ready();};
    tts_job.cancel_predicate = [this]() {return !runtime_ready();};
    tts_job.execute =
      [this, tts_runtime_ptr](const CancellationToken & token) {
        while (!token.stop_requested() && runtime_ready()) {
          tts_runtime_ptr->process_pending(
            [this](uint64_t epoch) {return epoch_is_current(epoch);},
            [this](JobModule module) {return module_enabled(module);},
            [this]() {return !runtime_ready();},
            std::chrono::steady_clock::now() + std::chrono::milliseconds(100),
            control_snapshot().epoch,
            control_snapshot().tts_enabled);
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
      };
    tts_job.commit_if_current =
      [this, epoch = tts_job.epoch, tts_module](
        const CancellationToken & token) {
        return try_commit(
          epoch, tts_module,
          std::chrono::steady_clock::now() + std::chrono::hours(24),
          [this]() {return !runtime_ready();}, token, [](){});
      };
    if (canonical_provider(tts_module) == ProviderClass::GuardedEp) {
      scheduler_->submit(std::move(tts_job));
    } else {
      cpu_worker(tts_module)->submit(std::move(tts_job));
    }
  }

  publish_state(true);
  return CallbackReturn::SUCCESS;
}

AiRuntimeNode::CallbackReturn AiRuntimeNode::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  runtime_ready_.store(false);
  worker_state_updates_enabled_.store(false);
  wakeword_queue_.stop();
  vad_asr_queue_.stop();
  wakeword_queue_.clear();
  vad_asr_queue_.clear();
  if (vision_runtime_) {
    vision_runtime_->clear_buffer();
  }
  if (tts_runtime_) {
    tts_runtime_->clear_pending();
  }
  if (scheduler_) {
    scheduler_->reject_new_jobs();
  }
  for (auto & entry : cpu_workers_) {
    entry.second->reject_new_jobs();
  }
  if (!stop_workers()) {
    set_last_error("worker stop exceeded stop_timeout_ms");
    publish_state(false, last_error());
    return CallbackReturn::ERROR;
  }
  publish_state(
    false, "runtime deactivated",
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
  return CallbackReturn::SUCCESS;
}

AiRuntimeNode::CallbackReturn AiRuntimeNode::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  runtime_ready_.store(false);
  worker_state_updates_enabled_.store(false);
  if (!stop_workers()) {
    set_last_error("worker stop exceeded stop_timeout_ms during cleanup");
    publish_state(false, last_error());
    return CallbackReturn::ERROR;
  }
  publish_state(
    false, "runtime cleanup",
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
  return release_resources() ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}

AiRuntimeNode::CallbackReturn AiRuntimeNode::on_error(
  const rclcpp_lifecycle::State &)
{
  runtime_ready_.store(false);
  worker_state_updates_enabled_.store(false);
  if (!stop_workers()) {
    set_last_error("worker stop exceeded stop_timeout_ms during error handling");
    publish_state(false, last_error());
    return CallbackReturn::FAILURE;
  }
  publish_state(
    false, last_error().empty() ? "runtime error" : last_error(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
  return release_resources() ? CallbackReturn::SUCCESS : CallbackReturn::FAILURE;
}

AiRuntimeNode::CallbackReturn AiRuntimeNode::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  runtime_ready_.store(false);
  worker_state_updates_enabled_.store(false);
  if (!stop_workers()) {
    set_last_error("worker stop exceeded stop_timeout_ms during shutdown");
    publish_state(false, last_error());
    return CallbackReturn::FAILURE;
  }
  publish_state(
    false, "runtime shutdown",
    lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED, "finalized");
  return release_resources() ? CallbackReturn::SUCCESS : CallbackReturn::FAILURE;
}

void AiRuntimeNode::on_control(const ControlMessage::SharedPtr message)
{
  update_control_snapshot(*message);
  RCLCPP_INFO(
    get_logger(),
    "[trace] runtime_control trace_id=%s epoch=%llu state=%s "
    "wake=%d vision=%d vad_asr=%d tts=%d reason=%s",
    message->trace_id.c_str(), static_cast<unsigned long long>(message->epoch),
    message->interaction_state_name.c_str(), message->wakeword_enabled,
    message->vision_enabled, message->vad_asr_enabled, message->tts_enabled,
    message->reason.c_str());
  // V2: map Supervisor interaction state to voice guard.
  if (runtime_core_) {
    runtime_core_->apply_control(control_snapshot());
  }
}

void AiRuntimeNode::publish_state(
  bool ready, const std::string & error,
  std::optional<uint8_t> lifecycle_state,
  const std::string & lifecycle_name)
{
  StateMessage message;
  const auto control = control_snapshot();
  message.header.stamp = now();
  message.trace_id = control.trace_id;
  message.epoch = control.epoch;
  message.runtime_ready = ready;
  if (lifecycle_state) {
    message.lifecycle_state = *lifecycle_state;
    message.lifecycle_state_name = lifecycle_name;
  } else if (ready) {
    message.lifecycle_state = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
    message.lifecycle_state_name = "active";
  } else {
    message.lifecycle_state = get_current_state().id();
    message.lifecycle_state_name = get_current_state().label();
  }
  message.mode = config_.backend_mode;
  if (scheduler_) {
    const auto stats = scheduler_->stats();
    message.active_job = stats.active_job;
    message.active_model = stats.active_job;
    message.queued_jobs = static_cast<uint32_t>(stats.queued);
  }
  for (const auto & entry : cpu_workers_) {
    const auto stats = entry.second->stats();
    message.queued_jobs += static_cast<uint32_t>(stats.queued);
    if (message.active_job.empty() && !stats.active_job.empty()) {
      message.active_job = stats.active_job;
      message.active_model = stats.active_job;
    }
  }
  message.last_error = error.empty() ? last_error() : error;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_published_state_ = message;
  }
  if (state_publisher_) {
    state_publisher_->publish(message);
  }
}

bool AiRuntimeNode::warmup_models()
{
  // models_ order: wakeword, vad_asr, vision, tts
  const std::vector<JobModule> modules{
    JobModule::WAKEWORD_CPU,
    JobModule::VAD_CPU,
    config_.vision_provider == "ep" ? JobModule::VISION_EP : JobModule::VISION_CPU,
    config_.tts_provider == "ep" ? JobModule::TTS_EP : JobModule::TTS_CPU,
  };
  for (std::size_t index = 0; index < models_.size(); ++index) {
    if (!warmup_model(*models_[index], modules[index])) {
      return false;
    }
  }
  // Warm up ASR separately with the correct module (EP or CPU).
  const auto asr_module = (config_.asr_provider == "ep")
    ? JobModule::ASR_EP : JobModule::ASR_CPU;
  if (!warmup_model(*vad_asr_runtime_, asr_module)) {
    return false;
  }
  return true;
}

bool AiRuntimeNode::warmup_model(ModelRuntime & model, JobModule module)
{
  const auto promise = std::make_shared<std::promise<std::string>>();
  const auto result_text = std::make_shared<std::string>();
  auto result = promise->get_future();
  InferenceJob job;
  job.id = "warmup:" + model.name();
  job.module = module;
  job.provider_class = canonical_provider(module);
  job.priority = canonical_priority(module);
  job.epoch = control_snapshot().epoch;
  job.deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(config_.job_deadline_ms);
  job.epoch_is_current = [this](uint64_t epoch) {return epoch_is_current(epoch);};
  job.module_enabled = [this](JobModule requested) {
      return !runtime_ready() || module_enabled(requested);
    };
  job.cancel_predicate = []() {return !rclcpp::ok();};
  job.execute = [&model, result_text, deadline = job.deadline](
    const CancellationToken & token) {
      try {
        model.warmup(token, deadline);
      } catch (const std::exception & error) {
        *result_text = error.what();
      }
    };
  job.commit_if_current =
    [this, promise, result_text, epoch = job.epoch, module, deadline = job.deadline,
    cancel = job.cancel_predicate](const CancellationToken & token) {
      return try_commit(
        epoch, module, deadline, cancel, token,
        [promise, result_text]() {promise->set_value(*result_text);});
    };

  const bool submitted = canonical_provider(module) == ProviderClass::GuardedEp ?
    scheduler_->submit(std::move(job)) : cpu_worker(module)->submit(std::move(job));
  if (!submitted) {
    set_last_error("failed to queue " + model.name() + " warmup");
    return false;
  }
  if (result.wait_for(std::chrono::milliseconds(config_.job_deadline_ms)) !=
    std::future_status::ready)
  {
    set_last_error(model.name() + " warmup timed out");
    return false;
  }
  const auto error = result.get();
  if (!error.empty()) {
    set_last_error(error);
    return false;
  }
  return true;
}

void AiRuntimeNode::start_workers()
{
  for (const auto module : kCpuStartOrder) {
    cpu_workers_.at(module)->start();
  }
}

bool AiRuntimeNode::stop_workers()
{
  bool stopped = true;
  for (auto iterator = kCpuStartOrder.rbegin(); iterator != kCpuStartOrder.rend(); ++iterator) {
    const auto worker = cpu_workers_.find(*iterator);
    if (worker != cpu_workers_.end()) {
      stopped = worker->second->stop() && stopped;
    }
  }
  if (scheduler_) {
    stopped = scheduler_->stop() && stopped;
  }
  return stopped;
}

bool AiRuntimeNode::release_resources()
{
  if (!stop_workers()) {
    set_last_error("cannot release resources while worker threads are still active");
    return false;
  }
  wakeword_queue_.clear();
  vad_asr_queue_.clear();
  for (auto & model : models_) {
    try {
      model->request_cancel();
      if (!model->stop(std::chrono::milliseconds(config_.stop_timeout_ms))) {
        set_last_error("model stop exceeded stop_timeout_ms: " + model->name());
      }
    } catch (const std::exception & e) {
      set_last_error(std::string("model stop threw: ") + e.what() +
                     " (" + model->name() + ")");
    }
  }
  if (scheduler_) {
    scheduler_->set_state_callback({});
  }
  for (auto & entry : cpu_workers_) {
    entry.second->set_state_callback({});
  }
  for (auto & model : models_) {
    try {
      model->final_join();
      if (model->loaded()) {
        model->unload();
      }
    } catch (const std::exception & e) {
      set_last_error(std::string("model unload threw: ") + e.what() +
                     " (" + model->name() + ")");
    }
  }
  models_.clear();
  mock_models_.clear();
  vision_runtime_.reset();
  tts_runtime_.reset();
  wakeword_runtime_.reset();
  vad_asr_runtime_.reset();
  audio_validator_.reset();
  audio_subscription_.reset();
  image_subscription_.reset();
  tts_text_subscription_.reset();
  detection_2d_publisher_.reset();
  tts_status_publisher_.reset();
  tts_play_publisher_.reset();
  control_subscription_.reset();
  model_data_callback_group_.reset();
  control_callback_group_.reset();
  cpu_workers_.clear();
  scheduler_.reset();
  guard_.reset();
  return true;
}

void AiRuntimeNode::set_last_error(const std::string & error)
{
  std::lock_guard<std::mutex> lock(error_mutex_);
  last_error_ = error;
}

void AiRuntimeNode::on_audio_frame(const AudioFrameMessage::SharedPtr message)
{
  const auto now = std::chrono::steady_clock::now();

  const auto result = audio_validator_->validate(
    message->sample_rate, message->channels, message->encoding,
    message->frame_ms, message->pcm_s16le, message->seq, now, now);

  if (!result.valid) {
    RCLCPP_ERROR(
      get_logger(), "audio frame validation failed: seq=%u reason=%s",
      message->seq, result.reason.c_str());
    reset_current_turn();
    return;
  }

  if (result.seq_gap) {
    // Seq gap: clear any backlog so stale frames don't flood VAD when it
    // re-enables, but do NOT reset the validator (seq tracking is already
    // updated) or the VAD segmenter (VAD worker detects the gap locally
    // and resets with pre-roll intact).
    RCLCPP_WARN(
      get_logger(), "audio frame seq gap at seq=%u. "
      "Clearing queues; VAD worker will handle segmenter reset locally.",
      message->seq);
    wakeword_queue_.clear();
    vad_asr_queue_.clear();
    // Fall through — current frame is valid, enqueue it normally.
  }

  AudioFrameData frame;
  frame.pcm = message->pcm_s16le;
  frame.samples = message->pcm_s16le.size();
  frame.trace_id = message->trace_id;
  frame.epoch = control_snapshot().epoch;
  frame.seq = message->seq;

  {
    AudioFrameData ww_frame = frame;
    if (!wakeword_queue_.push(std::move(ww_frame))) {
      RCLCPP_ERROR(get_logger(), "wakeword queue full, dropping frame seq=%u", message->seq);
      reset_current_turn();
      return;
    }
  }

  {
    AudioFrameData vad_frame = frame;
    if (!vad_asr_queue_.push(std::move(vad_frame))) {
      RCLCPP_ERROR(get_logger(), "vad_asr queue full, dropping frame seq=%u", message->seq);
      reset_current_turn();
      return;
    }
  }
}

void AiRuntimeNode::on_image_frame(const ImageMessage::SharedPtr message)
{
  if (!runtime_ready()) {
    return;
  }
  const auto control = control_snapshot();
  if (!control.vision_enabled) {
    return;
  }

  // V1 path (retained for compatibility).
  if (vision_runtime_) {
    vision_runtime_->put_frame(
      message->header.frame_id,
      message->width,
      message->height,
      message->encoding,
      control.trace_id,
      control.epoch,
      message->data.data(), message->data.size(),
      message->step);
  }

  // V2 path: feed VisionPipeline for multi-detector scheduling.
  if (runtime_core_) {
    VisionPipeline::FrameInput frame;
    frame.frame_id = message->header.frame_id;
    frame.width = message->width;
    frame.height = message->height;
    frame.encoding = message->encoding;
    frame.data.assign(message->data.begin(), message->data.end());
    frame.step = message->step;
    runtime_core_->vision().put_frame(frame);
  }
}

void AiRuntimeNode::on_tts_text_request(const TtsTextRequestMessage::SharedPtr message)
{
  if (!runtime_ready() || !tts_runtime_) {
    return;
  }
  if (message->trace_id.empty() || message->request_id.empty() || message->text.empty()) {
    RCLCPP_WARN(
      get_logger(), "tts text request rejected: missing trace_id, request_id, or text");
    return;
  }
  RCLCPP_INFO(
    get_logger(),
    "[trace] tts_request trace_id=%s request_id=%s epoch=%llu source=%s "
    "priority=%u text_len=%zu voice=%s",
    message->trace_id.c_str(), message->request_id.c_str(),
    static_cast<unsigned long long>(message->epoch), message->source.c_str(),
    static_cast<unsigned>(message->priority), message->text.size(),
    message->voice.c_str());
  tts_runtime_->submit_request(
    message->trace_id, message->request_id,
    message->epoch, message->source,
    message->priority, message->text, message->voice);
}

void AiRuntimeNode::reset_current_turn()
{
  wakeword_queue_.clear();
  vad_asr_queue_.clear();
  if (audio_validator_) {
    audio_validator_->reset();
  }
  if (wakeword_runtime_) {
    // WakewordRuntime has no turn state to reset beyond queue clearing.
  }
  if (vad_asr_runtime_) {
    vad_asr_runtime_->reset_turn();
  }
}

bool AiRuntimeNode::epoch_is_current(uint64_t epoch) const
{
  return commit_generation_gate_.epoch_is_current(epoch);
}

bool AiRuntimeNode::module_enabled(JobModule module) const
{
  if (!runtime_ready()) {
    return true;
  }
  return commit_generation_gate_.module_enabled(module);
}

bool AiRuntimeNode::try_commit(
  uint64_t epoch, JobModule module,
  std::chrono::steady_clock::time_point deadline,
  const std::function<bool()> & cancel_predicate,
  const CancellationToken & token,
  const std::function<void()> & commit)
{
  // External predicates are required to be pure atomic/non-blocking and run outside locks.
  if (cancel_predicate()) {
    return false;
  }
  return commit_generation_gate_.commit_if_current(
    epoch, module, deadline, token, commit);
}

void AiRuntimeNode::on_worker_state_changed()
{
  if (worker_state_updates_enabled_.load()) {
    publish_state(runtime_ready());
  }
}

BoundedCpuWorker * AiRuntimeNode::cpu_worker(JobModule module)
{
  const auto iterator = cpu_workers_.find(module);
  return iterator == cpu_workers_.end() ? nullptr : iterator->second.get();
}

}  // namespace k1muse_ai_runtime
