#include <gtest/gtest.h>

#include "k1muse_ai_runtime/runtime_core.hpp"

namespace k1muse_ai_runtime
{
namespace
{

// ── Test 1: Default state ──

TEST(RuntimeCoreTest, DefaultState)
{
  RuntimeCore core;
  EXPECT_FALSE(core.voice_guard().voice_active());
  EXPECT_TRUE(core.voice_guard().can_submit_vision());
  EXPECT_EQ(core.profile_mgr().current_mode(), RuntimeProfile::Mode::NORMAL);
}

// ── Test 2: Control → Voice guard mapping ──

TEST(RuntimeCoreTest, ControlListeningActivatesVoice)
{
  RuntimeCore core;

  ControlSnapshot ctrl;
  ctrl.interaction_state_name = "LISTENING";
  core.apply_control(ctrl);

  EXPECT_TRUE(core.voice_guard().voice_active());
  EXPECT_FALSE(core.voice_guard().can_submit_vision());
}

TEST(RuntimeCoreTest, ControlIdleReleasesVoice)
{
  RuntimeCore core;

  // First set LISTENING.
  ControlSnapshot listening;
  listening.interaction_state_name = "LISTENING";
  core.apply_control(listening);
  EXPECT_TRUE(core.voice_guard().voice_active());

  // Then set IDLE.
  ControlSnapshot idle;
  idle.interaction_state_name = "IDLE";
  core.apply_control(idle);
  EXPECT_FALSE(core.voice_guard().voice_active());
  EXPECT_TRUE(core.voice_guard().can_submit_vision());
}

TEST(RuntimeCoreTest, ControlTtsRunningActivatesVoice)
{
  RuntimeCore core;

  ControlSnapshot ctrl;
  ctrl.interaction_state_name = "TTS_RUNNING";
  core.apply_control(ctrl);

  EXPECT_TRUE(core.voice_guard().voice_active());
}

TEST(RuntimeCoreTest, ControlTargetingVisionActive)
{
  RuntimeCore core;

  // First set LISTENING.
  ControlSnapshot listening;
  listening.interaction_state_name = "LISTENING";
  core.apply_control(listening);
  EXPECT_TRUE(core.voice_guard().voice_active());

  // Then set TARGETING — vision active, voice released.
  ControlSnapshot targeting;
  targeting.interaction_state_name = "TARGETING";
  core.apply_control(targeting);
  EXPECT_FALSE(core.voice_guard().voice_active());
  EXPECT_TRUE(core.voice_guard().can_submit_vision());
}

// ── Test 3: Profile transition ──

TEST(RuntimeCoreTest, ProfileTransitionDrainWarmup)
{
  RuntimeCore core;

  // Request GUARD mode.
  bool ok = core.request_profile(RuntimeProfile::Mode::GUARD);
  EXPECT_TRUE(ok);

  // Advance: REQUESTED → APPLYING → DRAINING.
  core.advance_profile(); // → APPLYING
  core.advance_profile(); // → DRAINING

  // Signal drain complete.
  core.on_drain_complete();
  core.advance_profile(); // → WARMING

  // Signal warmup complete.
  core.on_warmup_complete();
  core.advance_profile(); // → COMMITTING
  core.advance_profile(); // → APPLIED

  EXPECT_EQ(core.profile_mgr().current_mode(),
            RuntimeProfile::Mode::GUARD);
}

// ── Test 4: Profile failure rollback ──

TEST(RuntimeCoreTest, ProfileFailureRollback)
{
  RuntimeCore core;

  core.request_profile(RuntimeProfile::Mode::GUARD);
  core.advance_profile(); // → APPLYING
  core.advance_profile(); // → DRAINING

  // Fail.
  core.on_profile_failure("pose detector load failed");

  EXPECT_EQ(core.profile_mgr().current_mode(),
            RuntimeProfile::Mode::NORMAL);
  EXPECT_EQ(core.profile_mgr().apply_state(),
            RuntimeProfileManager::ApplyState::APPLIED);
}

// ── Test 5: Emergency state leaves voice guard unchanged ──

TEST(RuntimeCoreTest, EmergencyLeavesVoiceUnchanged)
{
  RuntimeCore core;

  // Voice was not active before.
  ControlSnapshot emergency;
  emergency.interaction_state_name = "EMERGENCY_OR_FAULT";
  core.apply_control(emergency);

  // Should remain unchanged (not active).
  EXPECT_FALSE(core.voice_guard().voice_active());
}

}  // namespace
}  // namespace k1muse_ai_runtime
