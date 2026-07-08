#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "k1muse_vision_3d/vision_3d_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<k1muse_vision_3d::Vision3DNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
