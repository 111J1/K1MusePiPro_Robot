#include "k1muse_mock_devices/audio_scenario.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "k1muse_audio_msgs/msg/audio_frame.hpp"
#include "k1muse_common/qos_profiles.hpp"

namespace k1muse_mock_devices
{

class AudioScenarioNode : public rclcpp::Node
{
public:
  explicit AudioScenarioNode(const rclcpp::NodeOptions & options)
  : Node("audio_scenario_node", options)
  {
    // Declare parameters
    this->declare_parameter<std::string>("scenario", "wake_speech");
    this->declare_parameter<int>("publish_rate_ms", 20);
    this->declare_parameter<std::string>("trace_id", "scenario-test");

    // Read parameters
    auto scenario_str = this->get_parameter("scenario").as_string();
    publish_rate_ms_ = static_cast<uint16_t>(
      this->get_parameter("publish_rate_ms").as_int());
    trace_id_ = this->get_parameter("trace_id").as_string();

    // Build composite frame sequence
    frames_ = build_composite(scenario_str);
    frame_index_ = 0;

    // Publisher
    pub_ = this->create_publisher<k1muse_audio_msgs::msg::AudioFrame>(
      "/audio/raw_pcm", k1muse_common::qos::AudioStream(20));

    // Timer
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(publish_rate_ms_),
      [this]() { publish_next_frame(); });

    RCLCPP_INFO(this->get_logger(),
      "AudioScenarioNode started: scenario=%s frames=%zu rate=%ums trace=%s",
      scenario_str.c_str(), frames_.size(),
      static_cast<unsigned>(publish_rate_ms_), trace_id_.c_str());
  }

private:
  using FrameMsg = k1muse_audio_msgs::msg::AudioFrame;

  std::vector<AudioScenario::FrameData> frames_;
  std::size_t frame_index_{0};
  uint32_t seq_offset_{0};  // Incremented on each loop to avoid seq reuse.
  uint16_t publish_rate_ms_{20};
  std::string trace_id_;

  rclcpp::Publisher<FrameMsg>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  /// Build composite scenario from scenario name.
  std::vector<AudioScenario::FrameData> build_composite(
    const std::string & name)
  {
    AudioScenario::Config cfg;

    if (name == "wake_speech") {
      // WakeMarker(1) + Silence(5) + Speech(30) + Silence(25)
      std::vector<AudioScenario::FrameData> result;
      cfg.frames_per_scenario = 1;
      auto wake = AudioScenario(cfg).generate(
        AudioScenario::Type::WakeMarker, trace_id_);
      cfg.frames_per_scenario = 5;
      auto sil1 = AudioScenario(cfg).generate(
        AudioScenario::Type::Silence, trace_id_);
      cfg.frames_per_scenario = 30;
      auto speech = AudioScenario(cfg).generate(
        AudioScenario::Type::Speech, trace_id_);
      cfg.frames_per_scenario = 25;
      auto sil2 = AudioScenario(cfg).generate(
        AudioScenario::Type::Silence, trace_id_);

      result.insert(result.end(), wake.begin(), wake.end());
      result.insert(result.end(), sil1.begin(), sil1.end());
      result.insert(result.end(), speech.begin(), speech.end());
      result.insert(result.end(), sil2.begin(), sil2.end());
      renumber_seq(result);
      return result;

    } else if (name == "wake_only") {
      // WakeMarker(1) + Silence(49)
      std::vector<AudioScenario::FrameData> result;
      cfg.frames_per_scenario = 1;
      auto wake = AudioScenario(cfg).generate(
        AudioScenario::Type::WakeMarker, trace_id_);
      cfg.frames_per_scenario = 49;
      auto sil = AudioScenario(cfg).generate(
        AudioScenario::Type::Silence, trace_id_);
      result.insert(result.end(), wake.begin(), wake.end());
      result.insert(result.end(), sil.begin(), sil.end());
      renumber_seq(result);
      return result;

    } else if (name == "speech_only") {
      // Speech(50) + Silence(25)
      std::vector<AudioScenario::FrameData> result;
      cfg.frames_per_scenario = 50;
      auto speech = AudioScenario(cfg).generate(
        AudioScenario::Type::Speech, trace_id_);
      cfg.frames_per_scenario = 25;
      auto sil = AudioScenario(cfg).generate(
        AudioScenario::Type::Silence, trace_id_);
      result.insert(result.end(), speech.begin(), speech.end());
      result.insert(result.end(), sil.begin(), sil.end());
      renumber_seq(result);
      return result;

    } else if (name == "seq_gap") {
      // SeqGap: Speech(10 seq 1-10) + Speech(10 seq 20-29)
      cfg.frames_per_scenario = 20;
      return AudioScenario(cfg).generate(
        AudioScenario::Type::SeqGap, trace_id_);

    } else if (name == "no_speech") {
      // Silence(50)
      cfg.frames_per_scenario = 50;
      return AudioScenario(cfg).generate(
        AudioScenario::Type::NoSpeech, trace_id_);

    } else {
      RCLCPP_WARN(this->get_logger(),
        "Unknown scenario '%s', falling back to no_speech", name.c_str());
      cfg.frames_per_scenario = 50;
      return AudioScenario(cfg).generate(
        AudioScenario::Type::NoSpeech, trace_id_);
    }
  }

  /// Renumber seq to be contiguous starting from 1.
  static void renumber_seq(std::vector<AudioScenario::FrameData> & frames)
  {
    for (std::size_t i = 0; i < frames.size(); ++i) {
      frames[i].seq = static_cast<uint32_t>(i + 1);
    }
  }

  void publish_next_frame()
  {
    if (frame_index_ >= frames_.size()) {
      RCLCPP_INFO_ONCE(this->get_logger(), "All frames published, repeating.");
      // Increment seq_offset so the next loop's seq values don't reuse old ones.
      seq_offset_ += static_cast<uint32_t>(frames_.size());
      frame_index_ = 0;
    }

    const auto & fd = frames_[frame_index_++];
    auto msg = FrameMsg();

    msg.header.stamp = this->now();
    msg.header.frame_id = "audio_scenario";

    msg.trace_id = fd.trace_id;
    msg.seq = fd.seq + seq_offset_;  // Avoid seq reuse on loop restart.
    msg.sample_rate = fd.sample_rate;
    msg.channels = fd.channels;
    msg.encoding = fd.encoding;
    msg.frame_ms = fd.frame_ms;
    msg.pcm_s16le = fd.pcm;

    pub_->publish(msg);
  }
};

}  // namespace k1muse_mock_devices

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<k1muse_mock_devices::AudioScenarioNode>(
    rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
