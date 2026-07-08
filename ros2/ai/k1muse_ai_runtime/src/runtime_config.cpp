#include "k1muse_ai_runtime/runtime_config.hpp"

#include <array>
#include <limits>
#include <stdexcept>
#include <utility>

#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace k1muse_ai_runtime
{

RuntimeConfig RuntimeConfig::defaults()
{
  return RuntimeConfig{};
}

RuntimeConfig RuntimeConfig::declare_and_load(rclcpp_lifecycle::LifecycleNode & node)
{
  const auto defaults = RuntimeConfig::defaults();
  const auto string_parameter = [&node](const char * name, const std::string & value) {
      return node.has_parameter(name) ?
             node.get_parameter(name).as_string() :
             node.declare_parameter<std::string>(name, value);
    };
  const auto int_parameter = [&node](const char * name, int value) {
      const int64_t raw = node.has_parameter(name) ?
        node.get_parameter(name).as_int() :
        node.declare_parameter<int64_t>(name, static_cast<int64_t>(value));
      const auto checked = RuntimeConfig::checked_int(name, raw);
      if (!checked) {
        throw std::invalid_argument(std::string(name) + " is outside int range");
      }
      return *checked;
    };
  const auto bool_parameter = [&node](const char * name, bool value) {
      return node.has_parameter(name) ?
             node.get_parameter(name).as_bool() :
             node.declare_parameter<bool>(name, value);
    };
  RuntimeConfig config;
  config.backend_mode = string_parameter("backend_mode", defaults.backend_mode);
  config.guarded_queue_capacity =
    int_parameter("guarded_queue_capacity", defaults.guarded_queue_capacity);
  config.cpu_queue_capacity = int_parameter("cpu_queue_capacity", defaults.cpu_queue_capacity);
  config.job_deadline_ms = int_parameter("job_deadline_ms", defaults.job_deadline_ms);
  config.stop_timeout_ms = int_parameter("stop_timeout_ms", defaults.stop_timeout_ms);
  config.mock_warmup_delay_ms =
    int_parameter("mock_warmup_delay_ms", defaults.mock_warmup_delay_ms);
  config.mock_warmup_failure =
    bool_parameter("mock_warmup_failure", defaults.mock_warmup_failure);
  config.wakeword_provider =
    string_parameter("wakeword_provider", defaults.wakeword_provider);
  config.vad_provider = string_parameter("vad_provider", defaults.vad_provider);
  config.asr_provider = string_parameter("asr_provider", defaults.asr_provider);
  config.vision_provider = string_parameter("vision_provider", defaults.vision_provider);
  config.tts_provider = string_parameter("tts_provider", defaults.tts_provider);
  config.spacemit_ep_intra_threads =
    int_parameter(
    "spacemit_ep_intra_threads", defaults.spacemit_ep_intra_threads);
  config.spacemit_ep_inter_threads =
    int_parameter(
    "spacemit_ep_inter_threads", defaults.spacemit_ep_inter_threads);

  // T2: wakeword parameters
  const auto float_parameter = [&node](const char * name, float value) {
      return node.has_parameter(name) ?
             static_cast<float>(node.get_parameter(name).as_double()) :
             static_cast<float>(node.declare_parameter<double>(name, static_cast<double>(value)));
    };
  config.wakeword_threshold =
    float_parameter("wakeword_threshold", defaults.wakeword_threshold);
  config.wakeword_trigger_sample_value =
    int_parameter("wakeword_trigger_sample_value", defaults.wakeword_trigger_sample_value);

  // T2: VAD / ASR parameters
  config.vad_threshold =
    float_parameter("vad_threshold", defaults.vad_threshold);
  config.min_speech_ms = int_parameter("min_speech_ms", defaults.min_speech_ms);
  config.end_silence_ms = int_parameter("end_silence_ms", defaults.end_silence_ms);
  config.pre_roll_ms = int_parameter("pre_roll_ms", defaults.pre_roll_ms);
  config.post_pad_ms = int_parameter("post_pad_ms", defaults.post_pad_ms);
  config.max_utterance_ms =
    int_parameter("max_utterance_ms", defaults.max_utterance_ms);
  config.mock_asr_text =
    string_parameter("mock_asr_text", defaults.mock_asr_text);

  // T3: mock vision parameters
  config.mock_vision_detection_count =
    int_parameter("mock_vision_detection_count", defaults.mock_vision_detection_count);
  config.mock_vision_detection_class =
    string_parameter("mock_vision_detection_class", defaults.mock_vision_detection_class);
  config.mock_vision_detection_score =
    float_parameter("mock_vision_detection_score", defaults.mock_vision_detection_score);

  // T4: mock TTS parameters
  config.mock_tts_sample_rate =
    int_parameter("mock_tts_sample_rate", defaults.mock_tts_sample_rate);
  config.mock_tts_duration_ms =
    int_parameter("mock_tts_duration_ms", defaults.mock_tts_duration_ms);
  config.mock_tts_channels =
    int_parameter("mock_tts_channels", defaults.mock_tts_channels);

  // Real backend model paths
  config.sherpa_kws_model_path =
    string_parameter("sherpa_kws_model_path", defaults.sherpa_kws_model_path);
  config.sherpa_kws_keywords_path =
    string_parameter("sherpa_kws_keywords_path", defaults.sherpa_kws_keywords_path);
  config.vad_model_path =
    string_parameter("vad_model_path", defaults.vad_model_path);
  config.sensevoice_asr_model_path =
    string_parameter("sensevoice_asr_model_path", defaults.sensevoice_asr_model_path);
  config.vision_model_path =
    string_parameter("vision_model_path", defaults.vision_model_path);
  config.vision_config_path =
    string_parameter("vision_config_path", defaults.vision_config_path);
  config.vision_labels_path =
    string_parameter("vision_labels_path", defaults.vision_labels_path);
  config.tts_model_path =
    string_parameter("tts_model_path", defaults.tts_model_path);
  config.provider_fallback =
    string_parameter("provider_fallback", defaults.provider_fallback);

  return config;
}

std::optional<int> RuntimeConfig::checked_int(const std::string &, int64_t value)
{
  if (value < static_cast<int64_t>(std::numeric_limits<int>::min()) ||
    value > static_cast<int64_t>(std::numeric_limits<int>::max()))
  {
    return std::nullopt;
  }
  return static_cast<int>(value);
}

std::string RuntimeConfig::validate() const
{
  if (backend_mode != "mock" && backend_mode != "real") {
    return "backend_mode must be 'mock' or 'real'";
  }
#ifdef K1MUSE_ENABLE_REAL_K1_BACKENDS
  if (backend_mode == "real") {
    if (provider_fallback != "disabled" && provider_fallback != "ep_to_cpu") {
      return "provider_fallback must be 'disabled' or 'ep_to_cpu'";
    }
  }
#else
  if (backend_mode == "real") {
    return "backend_mode 'real' requires -DK1MUSE_ENABLE_REAL_K1_BACKENDS=ON";
  }
#endif
  if (guarded_queue_capacity <= 0) {
    return "guarded_queue_capacity must be greater than zero";
  }
  if (cpu_queue_capacity <= 0) {
    return "cpu_queue_capacity must be greater than zero";
  }
  if (job_deadline_ms <= 0) {
    return "job_deadline_ms must be greater than zero";
  }
  if (stop_timeout_ms <= 0) {
    return "stop_timeout_ms must be greater than zero";
  }
  if (mock_warmup_delay_ms < 0) {
    return "mock_warmup_delay_ms must not be negative";
  }
  if (spacemit_ep_intra_threads <= 0) {
    return "spacemit_ep_intra_threads must be greater than zero";
  }
  if (spacemit_ep_inter_threads <= 0) {
    return "spacemit_ep_inter_threads must be greater than zero";
  }
  const auto valid_provider = [](const std::string & provider) {
      return provider == "cpu" || provider == "ep";
    };
  for (const auto & entry : std::array<std::pair<const char *, const std::string *>, 3>{{
      {"asr_provider", &asr_provider},
      {"vision_provider", &vision_provider},
      {"tts_provider", &tts_provider},
    }})
  {
    if (!valid_provider(*entry.second)) {
      return std::string(entry.first) + " must be cpu or ep";
    }
  }
  if (wakeword_provider != "cpu") {
    return "wakeword_provider is fixed to cpu";
  }
  if (vad_provider != "cpu") {
    return "vad_provider is fixed to cpu";
  }
  // T2: wakeword validation
  if (wakeword_threshold < 0.0f || wakeword_threshold > 1.0f) {
    return "wakeword_threshold must be in [0.0, 1.0]";
  }
  // T2: VAD / ASR validation
  if (vad_threshold < 0.0f || vad_threshold > 1.0f) {
    return "vad_threshold must be in [0.0, 1.0]";
  }
  if (min_speech_ms <= 0) {
    return "min_speech_ms must be greater than zero";
  }
  if (end_silence_ms <= 0) {
    return "end_silence_ms must be greater than zero";
  }
  if (pre_roll_ms < 0) {
    return "pre_roll_ms must not be negative";
  }
  if (post_pad_ms < 0) {
    return "post_pad_ms must not be negative";
  }
  if (max_utterance_ms <= 0) {
    return "max_utterance_ms must be greater than zero";
  }
  // T3: mock vision validation
  if (mock_vision_detection_count < 0) {
    return "mock_vision_detection_count must not be negative";
  }
  if (mock_vision_detection_score < 0.0f || mock_vision_detection_score > 1.0f) {
    return "mock_vision_detection_score must be in [0.0, 1.0]";
  }
  // T4: mock TTS validation
  if (mock_tts_sample_rate <= 0) {
    return "mock_tts_sample_rate must be greater than zero";
  }
  if (mock_tts_duration_ms <= 0) {
    return "mock_tts_duration_ms must be greater than zero";
  }
  if (mock_tts_channels <= 0) {
    return "mock_tts_channels must be greater than zero";
  }
  return {};
}

}  // namespace k1muse_ai_runtime
