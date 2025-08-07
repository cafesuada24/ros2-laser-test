#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>

#include "ultrasonic_aggregator/AggregatorNode.h"

int main(int argc, char* argv[]) {

  rclcpp::init(argc, argv);


  auto aggregatorNode {
    std::make_shared<ultrasonic_aggregator::AggregatorNode>()
  };

  rclcpp::spin(aggregatorNode);
  rclcpp::shutdown();

  return 0;
}
