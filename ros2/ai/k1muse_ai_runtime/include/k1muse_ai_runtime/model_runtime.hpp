#pragma once

#include <chrono>
#include <string>

#include "k1muse_ai_runtime/inference_job.hpp"

namespace k1muse_ai_runtime
{

class ModelRuntime
{
public:
  using Deadline = std::chrono::steady_clock::time_point;

  // ModelRuntime owns no asynchronous execution threads. RuntimeScheduler or
  // BoundedCpuWorker owns every load, warmup, and inference execution context.
  // An adapter wrapping an internally threaded SDK must stop and final-join it.
  virtual ~ModelRuntime() = default;
  virtual const std::string & name() const = 0;
  virtual const std::string & provider() const = 0;
  // load/warmup must poll token and return no later than deadline.
  virtual void load(const CancellationToken & token, Deadline deadline) = 0;
  virtual void warmup(const CancellationToken & token, Deadline deadline) = 0;
  virtual void request_cancel() noexcept = 0;
  // A false result retains node resources; unload/destruction cannot proceed.
  virtual bool stop(std::chrono::milliseconds stop_timeout) noexcept = 0;
  // Final destruction barrier. Implementations must join any SDK-owned work.
  virtual void final_join() noexcept = 0;
  virtual void unload() noexcept = 0;
  virtual bool loaded() const noexcept = 0;
};

}  // namespace k1muse_ai_runtime
