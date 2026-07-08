#include "k1muse_ai_runtime/backends/real_vision_backend.hpp"

#ifdef K1MUSE_ENABLE_REAL_K1_BACKENDS

#include "vision_service.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace k1muse_ai_runtime
{

namespace
{

std::vector<std::string> load_labels(
  const std::string & config_path, const std::string & explicit_labels_path)
{
  // If an explicit labels path is provided, use it directly.
  // Otherwise derive labels.txt adjacent to the config file.
  // Format: one label per line.
  std::vector<std::string> labels;
  std::string labels_path = explicit_labels_path;
  if (labels_path.empty()) {
    labels_path = config_path;
    auto pos = labels_path.rfind('/');
    if (pos == std::string::npos) {
      pos = labels_path.rfind('\\');
    }
    if (pos != std::string::npos) {
      labels_path = labels_path.substr(0, pos + 1) + "labels.txt";
    } else {
      labels_path = "labels.txt";
    }
  }

  std::ifstream file(labels_path);
  if (file.is_open()) {
    std::string line;
    while (std::getline(file, line)) {
      if (!line.empty()) {
        labels.push_back(line);
      }
    }
  }
  return labels;
}

}  // namespace

struct RealVisionBackend::Impl
{
  std::string config_path;
  std::string model_path;
  std::string labels_path;
  float conf_threshold;
  float iou_threshold;
  bool is_loaded{false};

  std::unique_ptr<VisionService> service;
  std::vector<std::string> labels;

  Impl(const std::string & cfg, const std::string & model,
       const std::string & labels, float conf, float iou)
    : config_path(cfg), model_path(model), labels_path(labels),
      conf_threshold(conf), iou_threshold(iou)
  {
  }
};

RealVisionBackend::RealVisionBackend(
  const std::string & config_path,
  const std::string & model_path,
  const std::string & labels_path,
  float conf_threshold,
  float iou_threshold)
  : impl_(std::make_unique<Impl>(config_path, model_path, labels_path,
                                  conf_threshold, iou_threshold))
{
}

RealVisionBackend::~RealVisionBackend() = default;

const std::string & RealVisionBackend::name() const
{
  static const std::string n{"real_vision"};
  return n;
}

void RealVisionBackend::load()
{
  if (impl_->is_loaded) {
    return;
  }

  auto service =
    VisionService::Create(impl_->config_path, impl_->model_path);
  if (!service) {
    std::cerr << "[real_backend] real_vision load failed: "
              << VisionService::LastCreateError() << std::endl;
    return;
  }

  impl_->service = std::move(service);
  impl_->labels = load_labels(impl_->config_path, impl_->labels_path);
  impl_->is_loaded = true;
}

void RealVisionBackend::unload()
{
  if (impl_->service) {
    impl_->service->Release();
  }
  impl_->service.reset();
  impl_->is_loaded = false;
}

bool RealVisionBackend::loaded() const
{
  return impl_->is_loaded;
}

void RealVisionBackend::warmup()
{
  // No-op: VisionService handles warmup internally if configured.
}

VisionBackend::FrameResult RealVisionBackend::infer(
  const std::string & frame_id,
  uint32_t width, uint32_t height,
  const std::string & encoding,
  const uint8_t * data, size_t data_size,
  uint32_t step)
{
  FrameResult fr;
  fr.image_width = width;
  fr.image_height = height;

  if (!impl_->is_loaded || !impl_->service) {
    fr.reason = "Backend not loaded";
    return fr;
  }

  // Build cv::Mat from raw data if available.
  // step is the row stride in bytes; 0 means tightly packed (AUTO_STEP).
  const auto mat_step = static_cast<size_t>(step);
  cv::Mat image;
  if (data != nullptr && data_size > 0 && width > 0 && height > 0) {
    if (encoding == "bgr8" || encoding == "bgr") {
      image = cv::Mat(static_cast<int>(height), static_cast<int>(width),
                       CV_8UC3, const_cast<uint8_t *>(data), mat_step);
    } else if (encoding == "rgb8" || encoding == "rgb") {
      cv::Mat rgb(static_cast<int>(height), static_cast<int>(width),
                   CV_8UC3, const_cast<uint8_t *>(data), mat_step);
      cv::cvtColor(rgb, image, cv::COLOR_RGB2BGR);
    } else if (encoding == "mono8" || encoding == "gray") {
      image = cv::Mat(static_cast<int>(height), static_cast<int>(width),
                       CV_8UC1, const_cast<uint8_t *>(data), mat_step);
    } else {
      // Assume BGR8 as default fallback.
      image = cv::Mat(static_cast<int>(height), static_cast<int>(width),
                       CV_8UC3, const_cast<uint8_t *>(data), mat_step);
    }
  }

  if (image.empty()) {
    fr.reason = "No image data provided or unsupported encoding";
    return fr;
  }

  std::vector<VisionServiceResult> results;
  auto status = impl_->service->InferImage(
    image, &results, impl_->conf_threshold, impl_->iou_threshold);

  if (status != VISION_SERVICE_OK) {
    fr.reason = "InferImage failed: " + impl_->service->LastError();
    return fr;
  }

  fr.success = true;

  for (size_t i = 0; i < results.size(); ++i) {
    const auto & r = results[i];
    Detection det;

    // Convert (x1,y1,x2,y2) float to (x,y,w,h) uint32.
    det.x = static_cast<uint32_t>(std::max(0.0f, r.x1));
    det.y = static_cast<uint32_t>(std::max(0.0f, r.y1));
    det.width = static_cast<uint32_t>(
      std::max(0.0f, r.x2 - r.x1));
    det.height = static_cast<uint32_t>(
      std::max(0.0f, r.y2 - r.y1));
    det.score = r.score;

    // Convert label index to class name.
    if (r.label >= 0 &&
        static_cast<size_t>(r.label) < impl_->labels.size()) {
      det.class_name = impl_->labels[static_cast<size_t>(r.label)];
    } else {
      det.class_name = "object_" + std::to_string(r.label);
    }

    // Convert track_id to detection_id string.
    det.detection_id = std::to_string(r.track_id);

    fr.detections.push_back(std::move(det));
  }

  return fr;
}

void RealVisionBackend::reset()
{
  // VisionService does not have a separate reset; unload and reload.
}

}  // namespace k1muse_ai_runtime

#else  // !K1MUSE_ENABLE_REAL_K1_BACKENDS

namespace k1muse_ai_runtime
{

struct RealVisionBackend::Impl {};

RealVisionBackend::RealVisionBackend(
  const std::string &, const std::string &, const std::string &, float, float) {}

RealVisionBackend::~RealVisionBackend() = default;

const std::string & RealVisionBackend::name() const
{
  static const std::string n{"real_vision(stub)"};
  return n;
}

void RealVisionBackend::load() {}
void RealVisionBackend::unload() {}
bool RealVisionBackend::loaded() const { return false; }
void RealVisionBackend::warmup() {}

VisionBackend::FrameResult RealVisionBackend::infer(
  const std::string &, uint32_t, uint32_t,
  const std::string &, const uint8_t *, size_t, uint32_t)
{
  FrameResult fr;
  fr.reason = "Real backends not enabled";
  return fr;
}

void RealVisionBackend::reset() {}

}  // namespace k1muse_ai_runtime

#endif  // K1MUSE_ENABLE_REAL_K1_BACKENDS
