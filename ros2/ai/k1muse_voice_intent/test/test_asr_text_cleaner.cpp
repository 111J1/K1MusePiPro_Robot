#include <gtest/gtest.h>

#include "k1muse_voice_intent/asr_text_cleaner.hpp"

TEST(AsrTextCleaner, CleanSenseVoiceTags) {
  const std::string input = "<|zh|>前进<|NEUTRAL|>";
  const auto cleaned = k1muse_voice_intent::CleanAsrText(input);
  EXPECT_EQ(cleaned, "前进");
}

TEST(AsrTextCleaner, NormalizeWhitespace) {
  const std::string input = "hello   world";
  const auto cleaned = k1muse_voice_intent::CleanAsrText(input);
  EXPECT_EQ(cleaned, "hello world");
}

TEST(AsrTextCleaner, TrimWhitespace) {
  const std::string input = "  hello  ";
  const auto cleaned = k1muse_voice_intent::CleanAsrText(input);
  EXPECT_EQ(cleaned, "hello");
}

TEST(AsrTextCleaner, EmptyString) {
  const std::string input = "";
  const auto cleaned = k1muse_voice_intent::CleanAsrText(input);
  EXPECT_EQ(cleaned, "");
  EXPECT_FALSE(k1muse_voice_intent::IsMeaningfulText(cleaned));
}

TEST(AsrTextCleaner, OnlyTags) {
  const std::string input = "<|zh|><|Speech|>";
  const auto cleaned = k1muse_voice_intent::CleanAsrText(input);
  EXPECT_EQ(cleaned, "");
  EXPECT_FALSE(k1muse_voice_intent::IsMeaningfulText(cleaned));
}

TEST(AsrTextCleaner, MixedLanguage) {
  const std::string input = "hello<|zh|>你好";
  const auto cleaned = k1muse_voice_intent::CleanAsrText(input);
  EXPECT_EQ(cleaned, "hello你好");
}

TEST(AsrTextCleaner, IsMeaningfulText) {
  EXPECT_TRUE(k1muse_voice_intent::IsMeaningfulText("前进"));
  EXPECT_FALSE(k1muse_voice_intent::IsMeaningfulText(""));
}
