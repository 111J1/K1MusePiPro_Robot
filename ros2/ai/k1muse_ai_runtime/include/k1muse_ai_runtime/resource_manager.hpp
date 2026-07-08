#pragma once

#include <cstddef>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "k1muse_ai_runtime/model_runtime.hpp"

namespace k1muse_ai_runtime
{

/// Resource manager for model lifecycle (load/unload) and memory monitoring.
/// Allows registering models, loading/unloading them by name, and checking memory usage.
class ResourceManager
{
public:
  ResourceManager() = default;
  ~ResourceManager() = default;

  ResourceManager(const ResourceManager &) = delete;
  ResourceManager & operator=(const ResourceManager &) = delete;

  /// Register a model with a given name.
  /// If a model with the same name already exists, it will be replaced.
  void register_model(const std::string & name, std::unique_ptr<ModelRuntime> model);

  /// Unregister a model by name.
  /// Returns true if the model was found and removed, false otherwise.
  bool unregister_model(const std::string & name);

  /// Load a model by name.
  /// Throws if the model is not found or already loaded.
  void load_model(const std::string & name);

  /// Unload a model by name.
  /// Throws if the model is not found.
  void unload_model(const std::string & name);

  /// Check if a model is loaded.
  bool is_loaded(const std::string & name) const;

  /// Get a list of all registered model names.
  std::vector<std::string> registered_models() const;

  /// Get a list of all loaded model names.
  std::vector<std::string> loaded_models() const;

  /// Get the model runtime by name.
  /// Returns nullptr if not found.
  ModelRuntime * get_model(const std::string & name) const;

  /// Get the number of registered models.
  std::size_t size() const;

  /// Get the number of loaded models.
  std::size_t loaded_count() const;

  /// Get current memory usage in MB (reads from /proc/self/status on Linux).
  /// Returns 0.0 on unsupported platforms.
  float memory_usage_mb() const;

  /// Get total system memory in MB.
  /// Returns 0.0 on unsupported platforms.
  float total_memory_mb() const;

  /// Check if memory usage is above threshold.
  /// Returns true if memory usage exceeds the given threshold in MB.
  bool memory_above_threshold(float threshold_mb) const;

private:
  mutable std::mutex mutex_;
  struct ModelEntry
  {
    std::unique_ptr<ModelRuntime> model;
    bool loaded{false};
  };
  std::map<std::string, ModelEntry> models_;
};

}  // namespace k1muse_ai_runtime
