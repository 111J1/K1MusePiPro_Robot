#pragma once

#include <memory>
#include <string>

#include "k1muse_voice_intent/intent_types.hpp"
#include "k1muse_voice_intent/llama_server_client.hpp"
#include "k1muse_voice_intent/llm_intent_client.hpp"

namespace k1muse_voice_intent {

// Real LLM intent client — assembles the full pipeline:
//
//   user text → LlamaServerClient (HTTP)
//            → LlmResponseValidator (JSON validation)
//            → LlmIntentMapper (→ IntentDecision)
//
// Observes cancel / deadline via LlmRequestContext.
// Compiled only when K1MUSE_ENABLE_REAL_LLM_CLIENT=ON.
//
// When not compiled, the factory that creates this class will
// return nullptr and the node falls back to issuing a clear
// configuration error.
class RealLlmIntentClient : public LlmIntentClient {
 public:
  explicit RealLlmIntentClient(LlamaServerClient::Config config);
  ~RealLlmIntentClient() override;

  // LlmIntentClient interface
  bool health_check(std::string* reason = nullptr) override;
  bool warmup(std::string* reason = nullptr) override;
  LlmResult complete_intent(const std::string& text,
                            const LlmRequestContext& context) override;
  const std::string& name() const override;

  // Access the underlying transport (for testing with fake server).
  LlamaServerClient& transport();

 private:
  std::string name_;
  LlamaServerClient transport_;
};

}  // namespace k1muse_voice_intent
