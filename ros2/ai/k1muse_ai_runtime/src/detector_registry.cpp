#include "k1muse_ai_runtime/detector_registry.hpp"

#include <utility>

namespace k1muse_ai_runtime
{

void DetectorRegistry::register_detector(
  const std::string & name, std::unique_ptr<VisionBackend> backend)
{
  std::lock_guard<std::mutex> lock(mutex_);
  detectors_[name] = std::move(backend);
}

bool DetectorRegistry::unregister_detector(const std::string & name)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return detectors_.erase(name) > 0;
}

VisionBackend * DetectorRegistry::get_detector(const std::string & name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = detectors_.find(name);
  if (it == detectors_.end()) {
    return nullptr;
  }
  return it->second.get();
}

std::vector<std::string> DetectorRegistry::registered_detectors() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::string> names;
  names.reserve(detectors_.size());
  for (const auto & entry : detectors_) {
    names.push_back(entry.first);
  }
  return names;
}

bool DetectorRegistry::has_detector(const std::string & name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return detectors_.find(name) != detectors_.end();
}

std::size_t DetectorRegistry::size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return detectors_.size();
}

}  // namespace k1muse_ai_runtime
