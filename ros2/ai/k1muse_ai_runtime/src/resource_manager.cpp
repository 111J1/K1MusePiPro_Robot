#include "k1muse_ai_runtime/resource_manager.hpp"

#include <chrono>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <utility>

#ifdef __linux__
#include <sys/sysinfo.h>
#endif

namespace k1muse_ai_runtime
{

void ResourceManager::register_model(
  const std::string & name, std::unique_ptr<ModelRuntime> model)
{
  std::lock_guard<std::mutex> lock(mutex_);
  ModelEntry entry;
  entry.model = std::move(model);
  entry.loaded = false;
  models_[name] = std::move(entry);
}

bool ResourceManager::unregister_model(const std::string & name)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return models_.erase(name) > 0;
}

void ResourceManager::load_model(const std::string & name)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = models_.find(name);
  if (it == models_.end()) {
    throw std::runtime_error("Model not found: " + name);
  }
  if (it->second.loaded) {
    throw std::runtime_error("Model already loaded: " + name);
  }
  CancellationToken token;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
  it->second.model->load(token, deadline);
  it->second.loaded = true;
}

void ResourceManager::unload_model(const std::string & name)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = models_.find(name);
  if (it == models_.end()) {
    throw std::runtime_error("Model not found: " + name);
  }
  if (!it->second.loaded) {
    return;  // Already unloaded
  }
  it->second.model->unload();
  it->second.loaded = false;
}

bool ResourceManager::is_loaded(const std::string & name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = models_.find(name);
  if (it == models_.end()) {
    return false;
  }
  return it->second.loaded;
}

std::vector<std::string> ResourceManager::registered_models() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::string> names;
  names.reserve(models_.size());
  for (const auto & entry : models_) {
    names.push_back(entry.first);
  }
  return names;
}

std::vector<std::string> ResourceManager::loaded_models() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::string> names;
  for (const auto & entry : models_) {
    if (entry.second.loaded) {
      names.push_back(entry.first);
    }
  }
  return names;
}

ModelRuntime * ResourceManager::get_model(const std::string & name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = models_.find(name);
  if (it == models_.end()) {
    return nullptr;
  }
  return it->second.model.get();
}

std::size_t ResourceManager::size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return models_.size();
}

std::size_t ResourceManager::loaded_count() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::size_t count = 0;
  for (const auto & entry : models_) {
    if (entry.second.loaded) {
      ++count;
    }
  }
  return count;
}

float ResourceManager::memory_usage_mb() const
{
#ifdef __linux__
  std::ifstream status_file("/proc/self/status");
  if (!status_file.is_open()) {
    return 0.0f;
  }

  std::string line;
  while (std::getline(status_file, line)) {
    if (line.compare(0, 6, "VmRSS:") == 0) {
      std::istringstream iss(line);
      std::string key;
      long value_kb;
      std::string unit;
      iss >> key >> value_kb >> unit;
      return static_cast<float>(value_kb) / 1024.0f;
    }
  }
  return 0.0f;
#else
  return 0.0f;
#endif
}

float ResourceManager::total_memory_mb() const
{
#ifdef __linux__
  struct sysinfo si;
  if (sysinfo(&si) == 0) {
    return static_cast<float>(si.totalram) * si.mem_unit / (1024.0f * 1024.0f);
  }
  return 0.0f;
#else
  return 0.0f;
#endif
}

bool ResourceManager::memory_above_threshold(float threshold_mb) const
{
  return memory_usage_mb() > threshold_mb;
}

}  // namespace k1muse_ai_runtime
