#include "k1muse_ai_runtime/backend_factory.hpp"

#include "k1muse_ai_runtime/runtime_config.hpp"

// Mock backends (always available)
#include "k1muse_ai_runtime/backends/mock_asr_backend.hpp"
#include "k1muse_ai_runtime/backends/mock_tts_backend.hpp"
#include "k1muse_ai_runtime/backends/mock_vad_backend.hpp"
#include "k1muse_ai_runtime/backends/mock_vision_backend.hpp"
#include "k1muse_ai_runtime/backends/mock_wakeword_backend.hpp"

#ifdef K1MUSE_ENABLE_REAL_K1_BACKENDS
#include "k1muse_ai_runtime/backends/real_sherpa_wakeword_backend.hpp"
#include "k1muse_ai_runtime/backends/real_vad_backend.hpp"
#include "k1muse_ai_runtime/backends/real_sensevoice_asr_backend.hpp"
#include "k1muse_ai_runtime/backends/real_vision_backend.hpp"
#include "k1muse_ai_runtime/backends/real_tts_backend.hpp"
#endif

#include <iostream>
#include <stdexcept>

namespace k1muse_ai_runtime
{

namespace
{
void warn_fallback_not_implemented(const RuntimeConfig & config)
{
  static bool warned = false;
  if (!warned && config.backend_mode == "real" &&
    config.provider_fallback == "ep_to_cpu")
  {
    std::cerr << "[backend_factory] WARNING: provider_fallback='ep_to_cpu' is "
              << "configured but not yet implemented. The system will use the "
              << "configured provider as-is without automatic fallback."
              << std::endl;
    warned = true;
  }
}
}  // namespace

std::unique_ptr<WakewordBackend> create_wakeword_backend(const RuntimeConfig & config)
{
  warn_fallback_not_implemented(config);
  if (config.backend_mode == "mock") {
    auto backend = std::make_unique<MockWakewordBackend>();
    backend->set_trigger_sample_value(
      static_cast<int16_t>(config.wakeword_trigger_sample_value));
    return backend;
  }
#ifdef K1MUSE_ENABLE_REAL_K1_BACKENDS
  if (config.backend_mode == "real") {
    return std::make_unique<RealSherpaWakewordBackend>(
      config.sherpa_kws_model_path, config.wakeword_threshold,
      config.sherpa_kws_keywords_path);
  }
#endif
  throw std::runtime_error(
    "backend_mode '" + config.backend_mode + "' not available. "
    "Rebuild with -DK1MUSE_ENABLE_REAL_K1_BACKENDS=ON for real backends.");
}

std::unique_ptr<VadBackend> create_vad_backend(const RuntimeConfig & config)
{
  if (config.backend_mode == "mock") {
    return std::make_unique<MockVadBackend>();
  }
#ifdef K1MUSE_ENABLE_REAL_K1_BACKENDS
  if (config.backend_mode == "real") {
    return std::make_unique<RealVadBackend>(
      config.vad_model_path, config.vad_threshold);
  }
#endif
  throw std::runtime_error(
    "backend_mode '" + config.backend_mode + "' not available. "
    "Rebuild with -DK1MUSE_ENABLE_REAL_K1_BACKENDS=ON for real backends.");
}

std::unique_ptr<AsrBackend> create_asr_backend(const RuntimeConfig & config)
{
  if (config.backend_mode == "mock") {
    auto backend = std::make_unique<MockAsrBackend>();
    backend->set_mock_text(config.mock_asr_text);
    return backend;
  }
#ifdef K1MUSE_ENABLE_REAL_K1_BACKENDS
  if (config.backend_mode == "real") {
    return std::make_unique<RealSenseVoiceAsrBackend>(
      config.sensevoice_asr_model_path, "auto", config.asr_provider);
  }
#endif
  throw std::runtime_error(
    "backend_mode '" + config.backend_mode + "' not available. "
    "Rebuild with -DK1MUSE_ENABLE_REAL_K1_BACKENDS=ON for real backends.");
}

std::unique_ptr<VisionBackend> create_vision_backend(const RuntimeConfig & config)
{
  if (config.backend_mode == "mock") {
    auto backend = std::make_unique<MockVisionBackend>();
    backend->set_mock_detection_count(
      static_cast<uint32_t>(config.mock_vision_detection_count));
    backend->set_mock_detection_class(config.mock_vision_detection_class);
    backend->set_mock_detection_score(config.mock_vision_detection_score);
    return backend;
  }
#ifdef K1MUSE_ENABLE_REAL_K1_BACKENDS
  if (config.backend_mode == "real") {
    return std::make_unique<RealVisionBackend>(
      config.vision_config_path, config.vision_model_path,
      config.vision_labels_path);
  }
#endif
  throw std::runtime_error(
    "backend_mode '" + config.backend_mode + "' not available. "
    "Rebuild with -DK1MUSE_ENABLE_REAL_K1_BACKENDS=ON for real backends.");
}

std::unique_ptr<TtsBackend> create_tts_backend(const RuntimeConfig & config)
{
  if (config.backend_mode == "mock") {
    auto backend = std::make_unique<MockTtsBackend>();
    backend->set_mock_sample_rate(
      static_cast<uint32_t>(config.mock_tts_sample_rate));
    backend->set_mock_duration_ms(
      static_cast<uint32_t>(config.mock_tts_duration_ms));
    backend->set_mock_channels(
      static_cast<uint8_t>(config.mock_tts_channels));
    return backend;
  }
#ifdef K1MUSE_ENABLE_REAL_K1_BACKENDS
  if (config.backend_mode == "real") {
    return std::make_unique<RealTtsBackend>(
      config.tts_model_path, "matcha_zh_en", config.tts_provider);
  }
#endif
  throw std::runtime_error(
    "backend_mode '" + config.backend_mode + "' not available. "
    "Rebuild with -DK1MUSE_ENABLE_REAL_K1_BACKENDS=ON for real backends.");
}

}  // namespace k1muse_ai_runtime
