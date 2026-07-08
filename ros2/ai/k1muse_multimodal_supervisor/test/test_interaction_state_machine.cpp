#include "k1muse_multimodal_supervisor/interaction_state_machine.hpp"

#include <gtest/gtest.h>

using namespace k1muse_multimodal_supervisor;

// Helper: produce a context with matching active+event identity.
TransitionContext make_ctx(uint64_t epoch, const std::string& req_id = "",
                           const std::string& trace_id = "") {
  TransitionContext ctx;
  ctx.runtime_ready = true;
  ctx.audio_ready   = true;
  ctx.intent_ready  = true;
  ctx.active_epoch      = epoch;
  ctx.event_epoch       = epoch;
  ctx.active_request_id = req_id;
  ctx.event_request_id  = req_id;
  ctx.active_trace_id   = trace_id;
  ctx.event_trace_id    = trace_id;
  return ctx;
}

// ---------------------------------------------------------------------------
// 1. BootToIdle
// ---------------------------------------------------------------------------
TEST(InteractionStateMachine, BootToIdle) {
  InteractionStateMachine sm;
  ASSERT_EQ(sm.current_state(), State::BOOT);

  // Update readiness one-by-one, no transition yet.
  TransitionContext ctx;
  ctx.runtime_ready = true;
  ctx.audio_ready   = false;
  ctx.intent_ready  = false;
  auto r = sm.process(Event::RUNTIME_READY_CHANGED, ctx);
  EXPECT_FALSE(r.accepted);

  ctx.audio_ready = true;
  r = sm.process(Event::AUDIO_READY_CHANGED, ctx);
  EXPECT_FALSE(r.accepted);

  ctx.intent_ready = true;
  r = sm.process(Event::INTENT_READY_CHANGED, ctx);
  EXPECT_TRUE(r.accepted);
  EXPECT_EQ(sm.current_state(), State::IDLE);
}

// ---------------------------------------------------------------------------
// 2. IdleToWakeAck
// ---------------------------------------------------------------------------
TEST(InteractionStateMachine, IdleToWakeAck) {
  InteractionStateMachine sm;
  sm.set_state(State::IDLE);

  auto ctx = make_ctx(42);
  auto r = sm.process(Event::WAKEWORD_ACCEPTED, ctx);
  EXPECT_TRUE(r.accepted);
  EXPECT_EQ(sm.current_state(), State::WAKE_ACK);
}

// ---------------------------------------------------------------------------
// 3. WakeAckToListening
// ---------------------------------------------------------------------------
TEST(InteractionStateMachine, WakeAckToListening) {
  InteractionStateMachine sm;
  sm.set_state(State::WAKE_ACK);

  auto ctx = make_ctx(42, "req-1");
  auto r = sm.process(Event::WAKE_ACK_PLAYBACK_DONE, ctx);
  EXPECT_TRUE(r.accepted);
  EXPECT_EQ(sm.current_state(), State::LISTENING);
}

// ---------------------------------------------------------------------------
// 4. ListeningToIntentProcessing
// ---------------------------------------------------------------------------
TEST(InteractionStateMachine, ListeningToIntentProcessing) {
  InteractionStateMachine sm;
  sm.set_state(State::LISTENING);

  auto ctx = make_ctx(42, "req-1", "trace-1");
  auto r = sm.process(Event::LISTEN_RESULT_RECEIVED, ctx);
  EXPECT_TRUE(r.accepted);
  EXPECT_EQ(sm.current_state(), State::INTENT_PROCESSING);
}

// ---------------------------------------------------------------------------
// 5. IntentToTts
// ---------------------------------------------------------------------------
TEST(InteractionStateMachine, IntentToTts) {
  InteractionStateMachine sm;
  sm.set_state(State::INTENT_PROCESSING);

  auto ctx = make_ctx(42, "req-1");
  auto r = sm.process(Event::INTENT_FINISHED, ctx);
  EXPECT_TRUE(r.accepted);
  EXPECT_EQ(sm.current_state(), State::TTS_RUNNING);
}

// ---------------------------------------------------------------------------
// 6. TtsToIdle  (two-event: TTS_DONE + TTS_PLAYBACK_DONE)
// ---------------------------------------------------------------------------
TEST(InteractionStateMachine, TtsToIdle) {
  InteractionStateMachine sm;
  sm.set_state(State::TTS_RUNNING);

  auto ctx = make_ctx(42, "req-1");

  // First event -- should be held, not transition.
  auto r1 = sm.process(Event::TTS_DONE, ctx);
  EXPECT_FALSE(r1.accepted);
  EXPECT_EQ(sm.current_state(), State::TTS_RUNNING);

  // Second event -- triggers transition.
  auto r2 = sm.process(Event::TTS_PLAYBACK_DONE, ctx);
  EXPECT_TRUE(r2.accepted);
  EXPECT_EQ(sm.current_state(), State::IDLE);
}

// ---------------------------------------------------------------------------
// 6b. TtsToIdleReverse  (playback arrives before tts done)
// ---------------------------------------------------------------------------
TEST(InteractionStateMachine, TtsToIdleReverse) {
  InteractionStateMachine sm;
  sm.set_state(State::TTS_RUNNING);

  auto ctx = make_ctx(42, "req-1");

  auto r1 = sm.process(Event::TTS_PLAYBACK_DONE, ctx);
  EXPECT_FALSE(r1.accepted);

  auto r2 = sm.process(Event::TTS_DONE, ctx);
  EXPECT_TRUE(r2.accepted);
  EXPECT_EQ(sm.current_state(), State::IDLE);
}

// ---------------------------------------------------------------------------
// 7. NoSpeechTimeout
// ---------------------------------------------------------------------------
TEST(InteractionStateMachine, NoSpeechTimeout) {
  InteractionStateMachine sm;
  sm.set_state(State::LISTENING);

  TransitionContext ctx;
  auto r = sm.process(Event::NO_SPEECH_TIMEOUT, ctx);
  EXPECT_TRUE(r.accepted);
  EXPECT_EQ(sm.current_state(), State::IDLE);
}

// ---------------------------------------------------------------------------
// 8. StaleEpochRejected
// ---------------------------------------------------------------------------
TEST(InteractionStateMachine, StaleEpochRejected) {
  InteractionStateMachine sm;
  sm.set_state(State::IDLE);

  TransitionContext ctx;
  ctx.active_epoch = 10;
  ctx.event_epoch  = 9;   // stale
  auto r = sm.process(Event::WAKEWORD_ACCEPTED, ctx);
  EXPECT_FALSE(r.accepted);
  EXPECT_EQ(sm.current_state(), State::IDLE);
}

// ---------------------------------------------------------------------------
// 9. StaleRequestRejected
// ---------------------------------------------------------------------------
TEST(InteractionStateMachine, StaleRequestRejected) {
  InteractionStateMachine sm;
  sm.set_state(State::INTENT_PROCESSING);

  TransitionContext ctx;
  ctx.active_epoch      = 42;
  ctx.event_epoch       = 42;
  ctx.active_request_id = "req-current";
  ctx.event_request_id  = "req-old";    // stale
  auto r = sm.process(Event::INTENT_FINISHED, ctx);
  EXPECT_FALSE(r.accepted);
  EXPECT_EQ(sm.current_state(), State::INTENT_PROCESSING);
}

// ---------------------------------------------------------------------------
// 10. TargetingKeepsWakeword  (TARGETING state exists and is valid)
// ---------------------------------------------------------------------------
TEST(InteractionStateMachine, TargetingKeepsWakeword) {
  InteractionStateMachine sm;
  sm.set_state(State::IDLE);

  TransitionContext ctx;
  auto r = sm.process(Event::TARGET_REQUEST_RECEIVED, ctx);
  EXPECT_TRUE(r.accepted);
  EXPECT_EQ(sm.current_state(), State::TARGETING);

  // Verify we can return to IDLE.
  r = sm.process(Event::TARGET_RESPONSE_SENT, ctx);
  EXPECT_TRUE(r.accepted);
  EXPECT_EQ(sm.current_state(), State::IDLE);
}

// ---------------------------------------------------------------------------
// 11. FullVoiceFlow  BOOT -> IDLE -> WAKE_ACK -> LISTENING -> INTENT -> TTS -> IDLE
// ---------------------------------------------------------------------------
TEST(InteractionStateMachine, FullVoiceFlow) {
  InteractionStateMachine sm;
  ASSERT_EQ(sm.current_state(), State::BOOT);

  // BOOT -> IDLE
  TransitionContext ready_ctx;
  ready_ctx.runtime_ready = true;
  ready_ctx.audio_ready   = true;
  ready_ctx.intent_ready  = true;
  auto r = sm.process(Event::INTENT_READY_CHANGED, ready_ctx);
  EXPECT_TRUE(r.accepted);
  EXPECT_EQ(sm.current_state(), State::IDLE);

  const uint64_t epoch = 100;
  const std::string req = "req-full";
  const std::string trace = "trace-full";

  // IDLE -> WAKE_ACK
  auto ctx = make_ctx(epoch, req, trace);
  r = sm.process(Event::WAKEWORD_ACCEPTED, ctx);
  EXPECT_TRUE(r.accepted);
  EXPECT_EQ(sm.current_state(), State::WAKE_ACK);

  // WAKE_ACK -> LISTENING
  r = sm.process(Event::WAKE_ACK_PLAYBACK_DONE, ctx);
  EXPECT_TRUE(r.accepted);
  EXPECT_EQ(sm.current_state(), State::LISTENING);

  // LISTENING -> INTENT_PROCESSING
  r = sm.process(Event::LISTEN_RESULT_RECEIVED, ctx);
  EXPECT_TRUE(r.accepted);
  EXPECT_EQ(sm.current_state(), State::INTENT_PROCESSING);

  // INTENT_PROCESSING -> TTS_RUNNING
  r = sm.process(Event::INTENT_FINISHED, ctx);
  EXPECT_TRUE(r.accepted);
  EXPECT_EQ(sm.current_state(), State::TTS_RUNNING);

  // TTS_RUNNING -> IDLE  (both events, same request)
  r = sm.process(Event::TTS_DONE, ctx);
  EXPECT_FALSE(r.accepted);  // waiting for playback
  r = sm.process(Event::TTS_PLAYBACK_DONE, ctx);
  EXPECT_TRUE(r.accepted);
  EXPECT_EQ(sm.current_state(), State::IDLE);
}

// ---------------------------------------------------------------------------
// Additional coverage
// ---------------------------------------------------------------------------

TEST(InteractionStateMachine, SystemFaultFromAnyState) {
  for (auto s : {State::IDLE, State::WAKE_ACK, State::LISTENING,
                 State::INTENT_PROCESSING, State::TTS_RUNNING, State::TARGETING}) {
    InteractionStateMachine sm;
    sm.set_state(s);
    TransitionContext ctx;
    auto r = sm.process(Event::SYSTEM_FAULT, ctx);
    EXPECT_TRUE(r.accepted) << "failed from state " << state_name(s);
    EXPECT_EQ(sm.current_state(), State::EMERGENCY_OR_FAULT);
  }
}

TEST(InteractionStateMachine, FaultClearedNeedsRuntimeReady) {
  InteractionStateMachine sm;
  sm.set_state(State::EMERGENCY_OR_FAULT);

  TransitionContext ctx;
  ctx.runtime_ready = false;
  auto r = sm.process(Event::FAULT_CLEARED, ctx);
  EXPECT_FALSE(r.accepted);
  EXPECT_EQ(sm.current_state(), State::EMERGENCY_OR_FAULT);

  ctx.runtime_ready = true;
  r = sm.process(Event::FAULT_CLEARED, ctx);
  EXPECT_TRUE(r.accepted);
  EXPECT_EQ(sm.current_state(), State::IDLE);
}

TEST(InteractionStateMachine, IntentFailedWithFallback) {
  InteractionStateMachine sm;
  sm.set_state(State::INTENT_PROCESSING);

  TransitionContext ctx;
  ctx.has_fallback_tts = true;
  auto r = sm.process(Event::INTENT_FAILED, ctx);
  EXPECT_TRUE(r.accepted);
  EXPECT_EQ(sm.current_state(), State::TTS_RUNNING);
}

TEST(InteractionStateMachine, IntentFailedNoFallback) {
  InteractionStateMachine sm;
  sm.set_state(State::INTENT_PROCESSING);

  TransitionContext ctx;
  ctx.has_fallback_tts = false;
  auto r = sm.process(Event::INTENT_FAILED, ctx);
  EXPECT_TRUE(r.accepted);
  EXPECT_EQ(sm.current_state(), State::IDLE);
}

TEST(InteractionStateMachine, WakeAckPlaybackFailedFallback) {
  InteractionStateMachine sm;
  sm.set_state(State::WAKE_ACK);

  TransitionContext ctx;
  auto r = sm.process(Event::WAKE_ACK_PLAYBACK_FAILED, ctx);
  EXPECT_TRUE(r.accepted);
  EXPECT_EQ(sm.current_state(), State::LISTENING);
}

TEST(InteractionStateMachine, TtsFailedGoesToIdle) {
  InteractionStateMachine sm;
  sm.set_state(State::TTS_RUNNING);

  TransitionContext ctx;
  auto r = sm.process(Event::TTS_FAILED, ctx);
  EXPECT_TRUE(r.accepted);
  EXPECT_EQ(sm.current_state(), State::IDLE);
}

TEST(InteractionStateMachine, TtsPlaybackFailedGoesToIdle) {
  InteractionStateMachine sm;
  sm.set_state(State::TTS_RUNNING);

  TransitionContext ctx;
  auto r = sm.process(Event::TTS_PLAYBACK_FAILED, ctx);
  EXPECT_TRUE(r.accepted);
  EXPECT_EQ(sm.current_state(), State::IDLE);
}

TEST(InteractionStateMachine, WrongStateRejected) {
  InteractionStateMachine sm;
  sm.set_state(State::IDLE);

  TransitionContext ctx;
  // LISTEN_RESULT_RECEIVED should be rejected in IDLE.
  auto r = sm.process(Event::LISTEN_RESULT_RECEIVED, ctx);
  EXPECT_FALSE(r.accepted);
  EXPECT_EQ(sm.current_state(), State::IDLE);
}

TEST(InteractionStateMachine, UpdateReadiness) {
  InteractionStateMachine sm;
  EXPECT_EQ(sm.current_state(), State::BOOT);

  EXPECT_FALSE(sm.update_readiness(true, false, false));
  EXPECT_FALSE(sm.update_readiness(true, true, false));
  EXPECT_TRUE(sm.update_readiness(true, true, true));
}

TEST(InteractionStateMachine, StateName) {
  EXPECT_STREQ(state_name(State::BOOT), "BOOT");
  EXPECT_STREQ(state_name(State::IDLE), "IDLE");
  EXPECT_STREQ(state_name(State::WAKE_ACK), "WAKE_ACK");
  EXPECT_STREQ(state_name(State::LISTENING), "LISTENING");
  EXPECT_STREQ(state_name(State::INTENT_PROCESSING), "INTENT_PROCESSING");
  EXPECT_STREQ(state_name(State::TTS_RUNNING), "TTS_RUNNING");
  EXPECT_STREQ(state_name(State::TARGETING), "TARGETING");
  EXPECT_STREQ(state_name(State::EMERGENCY_OR_FAULT), "EMERGENCY_OR_FAULT");
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
