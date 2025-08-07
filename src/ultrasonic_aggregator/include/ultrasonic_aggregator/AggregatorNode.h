#ifndef ULTRASONIC_AGGREGATOR__AGGREGATORNODE_H_
#define ULTRASONIC_AGGREGATOR__AGGREGATORNODE_H_

#include <chrono>
#include <cstddef>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <vector>


namespace ultrasonic_aggregator {

using namespace std::chrono_literals;

class AggregatorNode : public rclcpp::Node {
 public:
  AggregatorNode();

 private:
  const std::string NODE_NAME {"ultrasonic_aggregator_node"};
  const std::string FRAME_NAME {"ultrasonic_frame"};
  const std::string PUBLISHER_TOPIC {"ultrasonic_scan"};
  const std::vector<std::string> SUBSCRIPTION_NODES{
      "input_scan_1",
      "input_scan_2",
      "input_scan_3",
  };
  const std::chrono::milliseconds PUBLISHING_FREQUENCY {50ms};

  void control_cycle();

  std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr>
      mScanSub{};
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr mScanPub{};

  std::vector<sensor_msgs::msg::LaserScan::UniquePtr> mLastScan{};
  void scanCallback(const size_t index,
                    sensor_msgs::msg::LaserScan::UniquePtr scan);

  rclcpp::TimerBase::SharedPtr mTimer {};
};
}  // namespace ultrasonic_aggregator

#endif  // ULTRASONIC_AGGREGATOR__AGGREGATORNODE_H_
