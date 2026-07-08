#include "k1muse_ai_runtime/backends/mock_pose_backend.hpp"

#include <sstream>
#include <thread>

namespace k1muse_ai_runtime
{

MockPoseBackend::MockPoseBackend() = default;

const std::string & MockPoseBackend::name() const
{
  static const std::string n = "mock_pose";
  return n;
}

void MockPoseBackend::load() { loaded_ = true; }
void MockPoseBackend::unload() { loaded_ = false; }
bool MockPoseBackend::loaded() const { return loaded_; }
void MockPoseBackend::warmup() {}
void MockPoseBackend::reset() {}

VisionBackend::FrameResult MockPoseBackend::infer(
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

  for (uint32_t i = 0; i < person_count_; ++i) {
    Detection d;
    std::ostringstream id_stream;
    id_stream << "pose_person_" << frame_id << "_" << i;
    d.detection_id = id_stream.str();
    d.class_name = "person";
    d.score = mock_score_;
    // Spread detections across the frame.
    d.x = static_cast<uint32_t>((i + 1) * width / (person_count_ + 1));
    d.y = height / 2;
    d.width = width / 10;
    d.height = height / 3;
    result.detections.push_back(d);
  }

  return result;
}

void MockPoseBackend::set_mock_person_count(uint32_t count)
{
  person_count_ = count;
}

void MockPoseBackend::set_mock_score(float score)
{
  mock_score_ = score;
}

void MockPoseBackend::set_mock_delay_ms(int ms)
{
  mock_delay_ms_ = ms;
}

}  // namespace k1muse_ai_runtime
