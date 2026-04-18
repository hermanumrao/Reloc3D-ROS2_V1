#include "rclcpp/rclcpp.hpp"
#include "relocalization_3d/relocalization_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RelocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
