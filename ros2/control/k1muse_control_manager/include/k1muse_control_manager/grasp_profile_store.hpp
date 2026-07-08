#pragma once

#include <map>
#include <optional>
#include <string>

#include "k1muse_control_core/grasp_profile.h"
#include "k1muse_control_core/placement_planner.h"

namespace k1muse_control_manager
{

struct GraspProfileSelection
{
  std::string object_name;
  std::string variant_name;
  k1_robot_geometry_t geometry{};
  k1_grasp_variant_t profile{};
  std::optional<k1_pose5f_t> carry_pose;
};

class GraspProfileStore
{
public:
  void load_from_file(const std::string & path);
  GraspProfileSelection resolve(
    const std::string & object_name,
    const std::string & variant_name = "") const;

  float default_open_angle_rad() const;

private:
  struct StoredVariant
  {
    k1_grasp_variant_t profile{};
    std::optional<k1_pose5f_t> carry_pose;
  };

  struct StoredObject
  {
    std::string default_variant;
    std::map<std::string, StoredVariant> variants;
  };

  k1_robot_geometry_t geometry_{};
  float gripper_min_rad_{0.0f};
  float gripper_max_rad_{0.0f};
  float default_open_angle_rad_{1.3f};
  std::map<std::string, StoredObject> objects_;
};

}  // namespace k1muse_control_manager
