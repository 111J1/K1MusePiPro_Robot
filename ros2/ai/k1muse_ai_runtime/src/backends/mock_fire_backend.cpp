#include "k1muse_ai_runtime/backends/mock_fire_backend.hpp"

#include <sstream>
#include <thread>

namespace k1muse_ai_runtime
{

MockFireBackend::MockFireBackend() = default;

const std::string & MockFireBackend::name() const
{
  static const std::string n = "mock_fire";
  return n;
}

void MockFireBackend::load() { loaded_ = true; }
void MockFireBackend::unload() { loaded_ = false; }
bool MockFireBackend::loaded() const { return loaded_; }
void MockFireBackend::warmup() {}

void MockFireBackend::reset()
{
  fire_present_ = false;
  smoke_present_ = false;
}

VisionBackend::FrameResult MockFireBackend::infer(
    const std::string & frame_id,
    uint32_t width, uint32_t height,
    const std::string & /*encoding*/,
    const uint8_t * /*data*/, size_t /*data_size*/,
    uint32_t /*step*/)
{
  if (mock_delay_ms_ > 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(mock_delay_ms_));
  }

  FrameResult result;
  result.success = true;
  result.image_width = width;
  result.image_height = height;

  if (fire_present_) {
    Detection d;
    std::ostringstream id_stream;
    id_stream << "fire_" << frame_id << "_0";
    d.detection_id = id_stream.str();
    d.class_name = "fire";
    d.score = mock_score_;
    d.x = width / 4;
    d.y = height / 3;
    d.width = width / 3;
    d.height = height / 3;
    result.detections.push_back(d);
  }

  if (smoke_present_) {
    Detection d;
    std::ostringstream id_stream;
    id_stream << "smoke_" << frame_id << "_0";
    d.detection_id = id_stream.str();
    d.class_name = "smoke";
    d.score = mock_score_ * 0.9f;
    d.x = width / 3;
    d.y = height / 4;
    d.width = width / 2;
    d.height = height / 2;
    result.detections.push_back(d);
  }

  return result;
}

void MockFireBackend::set_fire_present(bool present) { fire_present_ = present; }
void MockFireBackend::set_smoke_present(bool present) { smoke_present_ = present; }
void MockFireBackend::set_mock_score(float score) { mock_score_ = score; }
void MockFireBackend::set_mock_delay_ms(int ms) { mock_delay_ms_ = ms; }

}  // namespace k1muse_ai_runtime
