#include <filesystem>

#include <gtest/gtest.h>

#include "k1muse_control_manager/grasp_profile_store.hpp"

namespace
{

std::string profile_path()
{
  return (std::filesystem::path(TEST_SOURCE_DIR) /
    "config" / "grasp_profiles.yaml").string();
}

}  // namespace

TEST(GraspProfileStoreTest, LoadsCalibratedProfiles)
{
  k1muse_control_manager::GraspProfileStore store;
  store.load_from_file(profile_path());

  const auto umbrella = store.resolve("umbrella", "top");
  EXPECT_FLOAT_EQ(umbrella.profile.pitch_rad, 1.5f);
  EXPECT_FLOAT_EQ(umbrella.profile.gripper_close_rad, 0.30f);
  ASSERT_TRUE(umbrella.carry_pose.has_value());
  EXPECT_FLOAT_EQ(umbrella.carry_pose->pitch, 1.5f);

  const auto bottle = store.resolve("bottle");
  EXPECT_EQ(bottle.variant_name, "side");
  EXPECT_FLOAT_EQ(bottle.profile.gripper_close_rad, 0.38f);
  EXPECT_FLOAT_EQ(bottle.profile.lift_distance_m, 0.03f);
}
