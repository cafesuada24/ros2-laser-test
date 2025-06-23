#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include "fsm_bumpgo_cpp/BumpGoNode.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  
  RCLCPP_INFO(rclcpp::get_logger("main"), "TEST");

  const auto bumpgo_node {std::make_shared<fsm_bumpgo_cpp::BumpGoNode>()};
  rclcpp::spin(bumpgo_node);

  rclcpp::shutdown();
  return 0;
}
