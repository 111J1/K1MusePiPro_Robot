#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "k1muse_voice_intent/intent_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<k1muse_voice_intent::IntentNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
