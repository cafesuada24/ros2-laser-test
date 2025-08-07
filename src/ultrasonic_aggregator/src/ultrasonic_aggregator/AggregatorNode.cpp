// #include "../../include/ultrasonic_aggregator/AggregatorNode.h"
#include "ultrasonic_aggregator/AggregatorNode.h"

#include <algorithm>
#include <cstddef>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <utility>
#include <vector>

#include "sensor_msgs/msg/laser_scan.hpp"

namespace ultrasonic_aggregator {

AggregatorNode::AggregatorNode()
    : Node(NODE_NAME),
      mScanSub{},
      mScanPub{
          create_publisher<sensor_msgs::msg::LaserScan>(PUBLISHER_TOPIC, 100),
      },
      mTimer(create_wall_timer(50ms, [this]() { control_cycle(); })) {
  const auto numSubscriptions{SUBSCRIPTION_NODES.size()};
  mScanSub.reserve(numSubscriptions);
  mLastScan.resize(numSubscriptions);

  for (size_t i{0}; i < numSubscriptions; ++i) {
    mScanSub.push_back(create_subscription<sensor_msgs::msg::LaserScan>(
        SUBSCRIPTION_NODES[i], rclcpp::ServicesQoS(),
        [this, i](sensor_msgs::msg::LaserScan::UniquePtr scan) {
          scanCallback(i, std::move(scan));
        }));
  }
}

void AggregatorNode::scanCallback(const size_t index,
                                  sensor_msgs::msg::LaserScan::UniquePtr scan) {
  mLastScan[index] = std::move(scan);
}

void AggregatorNode::control_cycle() {
  sensor_msgs::msg::LaserScan publishingMsg{};

  publishingMsg.header.stamp = now();
  publishingMsg.header.frame_id = FRAME_NAME;

  std::vector<float> ranges{};
  for (const auto& scan : mLastScan) {
    if (scan == nullptr || scan->ranges.empty()) {
      continue;
    }
    const auto minDistanceScan{
        std::min_element(scan->ranges.begin(), scan->ranges.end())};
    ranges.push_back(*minDistanceScan);
  }

  if (!ranges.empty()) {
    publishingMsg.ranges = ranges;
    mScanPub->publish(publishingMsg);
  }
}
}  // namespace ultrasonic_aggregator
