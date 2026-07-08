#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "k1muse_ai_runtime/backends/vision_backend.hpp"

namespace k1muse_ai_runtime
{

/// Registry for managing multiple vision detector backends.
/// Allows registering, unregistering, and querying detectors by name.
class DetectorRegistry
{
public:
  DetectorRegistry() = default;
  ~DetectorRegistry() = default;

  DetectorRegistry(const DetectorRegistry &) = delete;
  DetectorRegistry & operator=(const DetectorRegistry &) = delete;

  /// Register a detector backend with a given name.
  /// If a detector with the same name already exists, it will be replaced.
  void register_detector(
    const std::string & name, std::unique_ptr<VisionBackend> backend);

  /// Unregister a detector by name.
  /// Returns true if the detector was found and removed, false otherwise.
  bool unregister_detector(const std::string & name);

  /// Get a detector backend by name.
  /// Returns nullptr if not found.
  VisionBackend * get_detector(const std::string & name) const;

  /// Get a list of all registered detector names.
  std::vector<std::string> registered_detectors() const;

  /// Check if a detector with the given name is registered.
  bool has_detector(const std::string & name) const;

  /// Get the number of registered detectors.
  std::size_t size() const;

private:
  mutable std::mutex mutex_;
  std::map<std::string, std::unique_ptr<VisionBackend>> detectors_;
};

}  // namespace k1muse_ai_runtime
