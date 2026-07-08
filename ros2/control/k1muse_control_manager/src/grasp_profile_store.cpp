#include "k1muse_control_manager/grasp_profile_store.hpp"

#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

namespace k1muse_control_manager
{

namespace
{

float require_float(const YAML::Node & node, const char * name)
{
  if (!node[name]) {
    throw std::runtime_error(std::string("missing field: ") + name);
  }
  return node[name].as<float>();
}

uint32_t require_bool_u32(const YAML::Node & node, const char * name)
{
  if (!node[name]) {
    throw std::runtime_error(std::string("missing field: ") + name);
  }
  return node[name].as<bool>() ? 1U : 0U;
}

int32_t parse_mode(const std::string & value)
{
  if (value == "side") {
    return K1_GRASP_MODE_SIDE;
  }
  if (value == "top") {
    return K1_GRASP_MODE_TOP;
  }
  if (value == "oblique") {
    return K1_GRASP_MODE_OBLIQUE;
  }
  throw std::runtime_error("unknown grasp mode: " + value);
}

int32_t parse_strategy(const std::string & value)
{
  if (value == "fixed_angle") {
    return K1_GRIPPER_FIXED_ANGLE;
  }
  if (value == "contact_switch") {
    return K1_GRIPPER_CONTACT_SWITCH;
  }
  if (value == "vision_verify") {
    return K1_GRIPPER_VISION_VERIFY;
  }
  throw std::runtime_error("unknown gripper strategy: " + value);
}

k1_pose5f_t parse_pose5(const YAML::Node & node, const char * name)
{
  if (!node || !node.IsSequence() || node.size() != 5U) {
    throw std::runtime_error(std::string(name) + " must be [x, y, z, roll, pitch]");
  }
  return k1_pose5f_t{
    node[0].as<float>(),
    node[1].as<float>(),
    node[2].as<float>(),
    node[3].as<float>(),
    node[4].as<float>()};
}

void assign_preferred_tcp(k1_grasp_variant_t & profile, const YAML::Node & node)
{
  if (!node || !node.IsSequence() || node.size() != 3U) {
    throw std::runtime_error("preferred_arm_tcp must be [x, y, z]");
  }
  profile.preferred_arm_x = node[0].as<float>();
  profile.preferred_arm_y = node[1].as<float>();
  profile.preferred_arm_z = node[2].as<float>();
}

k1_grasp_variant_t parse_variant(const YAML::Node & node)
{
  k1_grasp_variant_t profile{};
  profile.mode = parse_mode(node["mode"].as<std::string>());
  profile.gripper_strategy = parse_strategy(node["gripper_strategy"].as<std::string>());
  profile.calibrated = require_bool_u32(node, "calibrated");
  profile.gripper_open_rad = require_float(node, "gripper_open_rad");
  profile.gripper_close_rad = require_float(node, "gripper_close_rad");
  profile.roll_rad = require_float(node, "roll_rad");
  profile.pitch_rad = require_float(node, "pitch_rad");
  assign_preferred_tcp(profile, node["preferred_arm_tcp"]);
  profile.approach_distance_m = require_float(node, "approach_distance_m");
  profile.retreat_distance_m = require_float(node, "retreat_distance_m");
  profile.lift_distance_m = require_float(node, "lift_distance_m");
  profile.transport_max_v = require_float(node, "transport_max_v");
  profile.transport_max_omega = require_float(node, "transport_max_omega");
  profile.settle_time_ms = node["settle_time_ms"].as<uint32_t>();
  return profile;
}

}  // namespace

void GraspProfileStore::load_from_file(const std::string & path)
{
  const YAML::Node root = YAML::LoadFile(path);

  const auto geometry = root["robot_geometry"];
  if (!geometry) {
    throw std::runtime_error("missing robot_geometry");
  }
  geometry_.arm_mount_x = require_float(geometry, "arm_mount_x");
  geometry_.arm_mount_y = require_float(geometry, "arm_mount_y");
  geometry_.arm_base_z_at_lift_zero = require_float(geometry, "arm_base_z_at_lift_zero");
  geometry_.lift_min_z = require_float(geometry, "lift_min_z");
  geometry_.lift_max_z = require_float(geometry, "lift_max_z");

  const auto limits = root["gripper_limits"];
  if (!limits) {
    throw std::runtime_error("missing gripper_limits");
  }
  gripper_min_rad_ = require_float(limits, "min_rad");
  gripper_max_rad_ = require_float(limits, "max_rad");

  const auto objects = root["objects"];
  if (!objects || !objects.IsMap()) {
    throw std::runtime_error("missing objects");
  }

  objects_.clear();
  bool default_open_angle_set = false;
  for (const auto & object_entry : objects) {
    const std::string object_name = object_entry.first.as<std::string>();
    const auto object_node = object_entry.second;

    StoredObject stored_object;
    stored_object.default_variant = object_node["default_variant"].as<std::string>();

    const auto variants = object_node["variants"];
    if (!variants || !variants.IsMap()) {
      throw std::runtime_error("missing variants for object: " + object_name);
    }

    for (const auto & variant_entry : variants) {
      const std::string variant_name = variant_entry.first.as<std::string>();
      const auto variant_node = variant_entry.second;

      StoredVariant stored_variant;
      stored_variant.profile = parse_variant(variant_node);
      const auto status = k1_grasp_profile_validate(
        &stored_variant.profile, gripper_min_rad_, gripper_max_rad_);
      if (status != K1_PROFILE_OK) {
        throw std::runtime_error(
          "invalid grasp profile " + object_name + "/" + variant_name +
          ", status=" + std::to_string(static_cast<int>(status)));
      }
      if (variant_node["carry_pose"]) {
        stored_variant.carry_pose = parse_pose5(variant_node["carry_pose"], "carry_pose");
      }
      if (!default_open_angle_set) {
        default_open_angle_rad_ = stored_variant.profile.gripper_open_rad;
        default_open_angle_set = true;
      }
      stored_object.variants.emplace(variant_name, stored_variant);
    }

    if (stored_object.variants.find(stored_object.default_variant) ==
        stored_object.variants.end()) {
      throw std::runtime_error(
        "default variant not found for object: " + object_name);
    }
    objects_.emplace(object_name, stored_object);
  }
}

GraspProfileSelection GraspProfileStore::resolve(
  const std::string & object_name,
  const std::string & variant_name) const
{
  const auto object_it = objects_.find(object_name);
  if (object_it == objects_.end()) {
    throw std::runtime_error("unknown grasp object: " + object_name);
  }
  const auto & object = object_it->second;
  const std::string selected_variant =
    variant_name.empty() ? object.default_variant : variant_name;
  const auto variant_it = object.variants.find(selected_variant);
  if (variant_it == object.variants.end()) {
    throw std::runtime_error(
      "unknown grasp variant: " + object_name + "/" + selected_variant);
  }

  GraspProfileSelection selection;
  selection.object_name = object_name;
  selection.variant_name = selected_variant;
  selection.geometry = geometry_;
  selection.profile = variant_it->second.profile;
  selection.carry_pose = variant_it->second.carry_pose;
  return selection;
}

float GraspProfileStore::default_open_angle_rad() const
{
  return default_open_angle_rad_;
}

}  // namespace k1muse_control_manager
