#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "k1muse_ai_runtime/backends/wakeword_backend.hpp"
#include "k1muse_ai_runtime/control_snapshot.hpp"
#include "k1muse_ai_runtime/model_runtime.hpp"

namespace k1muse_ai_runtime
{

class WakewordRuntime final : public ModelRuntime
{
public:
  using WakewordCallback = std::function<void(
    const std::string & trace_id, uint64_t epoch,
    float confidence, const std::string & keyword)>;

  WakewordRuntime(
    std::unique_ptr<WakewordBackend> backend,
    WakewordCallback callback);

  const std::string & name() const override;
  const std::string & provider() const override;

  void load(const CancellationToken & token, Deadline deadline) override;
  void warmup(const CancellationToken & token, Deadline deadline) override;

  void request_cancel() noexcept override;
  bool stop(std::chrono::milliseconds stop_timeout) noexcept override;
  void final_join() noexcept override;
  void unload() noexcept override;
  bool loaded() const noexcept override;

  /// Process one audio chunk. Checks wakeword_enabled from control; if
  /// detection fires, invokes the wakeword callback.
  void process_audio(
    const int16_t * pcm, size_t samples,
    const std::string & trace_id, uint64_t epoch,
    const ControlSnapshot & control);

private:
  std::unique_ptr<WakewordBackend> backend_;
  WakewordCallback callback_;
  std::atomic<bool> stop_requested_{false};
};

}  // namespace k1muse_ai_runtime
