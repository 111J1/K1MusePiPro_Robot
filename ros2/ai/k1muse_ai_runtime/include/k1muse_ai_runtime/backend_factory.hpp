#pragma once

#include <memory>
#include <string>

namespace k1muse_ai_runtime
{

class WakewordBackend;
class VadBackend;
class AsrBackend;
class VisionBackend;
class TtsBackend;
struct RuntimeConfig;

/// Factory functions -- one per backend type.
/// Returns a mock or real backend depending on \c config.backend_mode.
/// Throws std::runtime_error if backend_mode is "real" but real backends
/// were not compiled in (K1MUSE_ENABLE_REAL_K1_BACKENDS is OFF).
std::unique_ptr<WakewordBackend> create_wakeword_backend(const RuntimeConfig & config);
std::unique_ptr<VadBackend> create_vad_backend(const RuntimeConfig & config);
std::unique_ptr<AsrBackend> create_asr_backend(const RuntimeConfig & config);
std::unique_ptr<VisionBackend> create_vision_backend(const RuntimeConfig & config);
std::unique_ptr<TtsBackend> create_tts_backend(const RuntimeConfig & config);

}  // namespace k1muse_ai_runtime
