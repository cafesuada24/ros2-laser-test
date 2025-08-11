#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "vff_avoidance/AvoidanceNode.h"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto avoidance_node{std::make_shared<vff_avoidance::AvoidanceNode>()};
  rclcpp::spin(avoidance_node);
  rclcpp::shutdown();
  return 0;
}
