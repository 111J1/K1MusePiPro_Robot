#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "k1muse_task_manager/task_manager_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<k1muse_task_manager::TaskManagerNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
