#include "k1muse_ai_runtime/backends/mock_vision_backend.hpp"

namespace k1muse_ai_runtime
{

MockVisionBackend::MockVisionBackend() = default;

const std::string & MockVisionBackend::name() const
{
  static const std::string n{"mock_vision"};
  return n;
}

void MockVisionBackend::load()
{
  loaded_ = true;
}

void MockVisionBackend::unload()
{
  loaded_ = false;
}

bool MockVisionBackend::loaded() const
{
  return loaded_;
}

void MockVisionBackend::warmup()
{
  // No-op for mock.
}

VisionBackend::FrameResult MockVisionBackend::infer(
  const std::string & frame_id,
  uint32_t width, uint32_t height,
  const std::string & /*encoding*/,
  const uint8_t * /*data*/, size_t /*data_size*/,
  uint32_t /*step*/)
{
  FrameResult result;
  result.image_width = width;
  result.image_height = height;

  if (!loaded_) {
    result.reason = "backend not loaded";
    return result;
  }

  if (width == 0 || height == 0) {
    result.reason = "invalid image dimensions";
    return result;
  }

  // Generate deterministic detections centered in the image.
  result.success = true;
  result.detections.reserve(mock_detection_count_);

  for (uint32_t i = 0; i < mock_detection_count_; ++i) {
    Detection det;
    det.detection_id = frame_id + "_det_" + std::to_string(i);
    det.class_name = mock_detection_class_;
    det.score = mock_detection_score_;

    // Place detections centered with deterministic size.
    det.width = width / 4;
    det.height = height / 4;
    det.x = (width - det.width) / 2;
    det.y = (height - det.height) / 2;

    result.detections.push_back(std::move(det));
  }

  return result;
}

void MockVisionBackend::reset()
{
  // Nothing to reset beyond load state for the mock.
}

void MockVisionBackend::set_mock_detection_count(uint32_t count)
{
  mock_detection_count_ = count;
}

void MockVisionBackend::set_mock_detection_class(const std::string & class_name)
{
  mock_detection_class_ = class_name;
}

void MockVisionBackend::set_mock_detection_score(float score)
{
  mock_detection_score_ = score;
}

}  // namespace k1muse_ai_runtime
