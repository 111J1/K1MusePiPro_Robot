#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace rclcpp_lifecycle
{
class LifecycleNode;
}

namespace k1muse_ai_runtime
{

struct RuntimeConfig
{
  std::string backend_mode{"mock"};
  int guarded_queue_capacity{16};
  int cpu_queue_capacity{16};
  int job_deadline_ms{3000};
  int stop_timeout_ms{1000};
  int mock_warmup_delay_ms{0};
  bool mock_warmup_failure{false};
  std::string wakeword_provider{"cpu"};
  std::string vad_provider{"cpu"};
  std::string asr_provider{"ep"};
  std::string vision_provider{"ep"};
  std::string tts_provider{"cpu"};
  int spacemit_ep_intra_threads{2};
  int spacemit_ep_inter_threads{1};

  // T2: wakeword
  float wakeword_threshold{0.25f};
  int wakeword_trigger_sample_value{0x7FFF};

  // T2: VAD / ASR
  float vad_threshold{0.5f};
  int min_speech_ms{250};
  int end_silence_ms{500};
  int pre_roll_ms{300};
  int post_pad_ms{150};
  int max_utterance_ms{10000};
  std::string mock_asr_text{"mock transcription"};

  // T3: mock vision
  int mock_vision_detection_count{1};
  std::string mock_vision_detection_class{"object"};
  float mock_vision_detection_score{0.9f};

  // T4: mock TTS
  int mock_tts_sample_rate{16000};
  int mock_tts_duration_ms{500};
  int mock_tts_channels{1};

  // Real backend model paths (only used when backend_mode == "real")
  std::string sherpa_kws_model_path;
  std::string sherpa_kws_keywords_path;  // must be set in YAML for real backend
  std::string vad_model_path;
  std::string sensevoice_asr_model_path;
  std::string vision_model_path;
  std::string vision_config_path;
  std::string vision_labels_path;  // explicit labels file; empty = derive from config_path
  std::string tts_model_path;
  std::string provider_fallback{"disabled"};  // "disabled" or "ep_to_cpu"

  static RuntimeConfig defaults();
  static RuntimeConfig declare_and_load(rclcpp_lifecycle::LifecycleNode & node);
  static std::optional<int> checked_int(const std::string & name, int64_t value);
  std::string validate() const;
};

}  // namespace k1muse_ai_runtime
