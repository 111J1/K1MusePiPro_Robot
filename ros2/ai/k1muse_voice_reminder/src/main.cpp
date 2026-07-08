#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "k1muse_voice_reminder/reminder_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<k1muse_voice_reminder::ReminderNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
