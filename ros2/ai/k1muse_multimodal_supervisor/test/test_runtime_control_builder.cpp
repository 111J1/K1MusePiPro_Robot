#include "k1muse_multimodal_supervisor/runtime_control_builder.hpp"

#include <gtest/gtest.h>

using namespace k1muse_multimodal_supervisor;

// ---------------------------------------------------------------------------
// 1. IdleFlags
// ---------------------------------------------------------------------------
TEST(RuntimeControlBuilder, IdleFlags) {
  auto f = RuntimeControlBuilder::flags_for_state(State::IDLE);
  EXPECT_TRUE(f.wakeword_enabled);
  EXPECT_TRUE(f.vision_enabled);
  EXPECT_FALSE(f.vad_asr_enabled);
  EXPECT_FALSE(f.tts_enabled);
}

// ---------------------------------------------------------------------------
// 2. WakeAckFlags
// ---------------------------------------------------------------------------
TEST(RuntimeControlBuilder, WakeAckFlags) {
  auto f = RuntimeControlBuilder::flags_for_state(State::WAKE_ACK);
  EXPECT_FALSE(f.wakeword_enabled);
  EXPECT_FALSE(f.vision_enabled);
  EXPECT_FALSE(f.vad_asr_enabled);
  EXPECT_FALSE(f.tts_enabled);
}

// ---------------------------------------------------------------------------
// 3. ListeningFlags
// ---------------------------------------------------------------------------
TEST(RuntimeControlBuilder, ListeningFlags) {
  auto f = RuntimeControlBuilder::flags_for_state(State::LISTENING);
  EXPECT_FALSE(f.wakeword_enabled);
  EXPECT_FALSE(f.vision_enabled);
  EXPECT_TRUE(f.vad_asr_enabled);
  EXPECT_FALSE(f.tts_enabled);
}

// ---------------------------------------------------------------------------
// 4. IntentProcessingFlags
// ---------------------------------------------------------------------------
TEST(RuntimeControlBuilder, IntentProcessingFlags) {
  auto f = RuntimeControlBuilder::flags_for_state(State::INTENT_PROCESSING);
  EXPECT_FALSE(f.wakeword_enabled);
  EXPECT_FALSE(f.vision_enabled);
  EXPECT_FALSE(f.vad_asr_enabled);
  EXPECT_FALSE(f.tts_enabled);
}

// ---------------------------------------------------------------------------
// 5. TtsRunningFlags
// ---------------------------------------------------------------------------
TEST(RuntimeControlBuilder, TtsRunningFlags) {
  auto f = RuntimeControlBuilder::flags_for_state(State::TTS_RUNNING);
  EXPECT_FALSE(f.wakeword_enabled);
  EXPECT_FALSE(f.vision_enabled);
  EXPECT_FALSE(f.vad_asr_enabled);
  EXPECT_TRUE(f.tts_enabled);
}

// ---------------------------------------------------------------------------
// 6. TargetingFlags
// ---------------------------------------------------------------------------
TEST(RuntimeControlBuilder, TargetingFlags) {
  auto f = RuntimeControlBuilder::flags_for_state(State::TARGETING);
  EXPECT_TRUE(f.wakeword_enabled);
  EXPECT_TRUE(f.vision_enabled);
  EXPECT_FALSE(f.vad_asr_enabled);
  EXPECT_FALSE(f.tts_enabled);
}

// ---------------------------------------------------------------------------
// 7. EmergencyFlags
// ---------------------------------------------------------------------------
TEST(RuntimeControlBuilder, EmergencyFlags) {
  auto f = RuntimeControlBuilder::flags_for_state(State::EMERGENCY_OR_FAULT);
  EXPECT_FALSE(f.wakeword_enabled);
  EXPECT_FALSE(f.vision_enabled);
  EXPECT_FALSE(f.vad_asr_enabled);
  EXPECT_FALSE(f.tts_enabled);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
