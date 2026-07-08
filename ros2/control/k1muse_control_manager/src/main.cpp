#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "k1muse_control_manager/control_manager_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<k1muse_control_manager::ControlManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
