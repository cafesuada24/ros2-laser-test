#ifndef TF2_DETECTOR_OBSTACLEDETECTORIMPROVED_H
#define TF2_DETECTOR_OBSTACLEDETECTORIMPROVED_H

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "sensor_msgs/msg/laser_scan.hpp"

namespace tf2_detector {
class ObstacleDetectorNode : public rclcpp::Node {
 public:
  ObstacleDetectorNode();

 private:
  static inline const std::string NODE_NAME {"obstacle_detector"};
  std::vector<std::string> inputScans {};
  std::vector<std::string> scannerFrames {};

  void scan_callback(const size_t scannerIndex, sensor_msgs::msg::LaserScan::UniquePtr msg);

  std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> mScanSub {};
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> m_tf_broadcaster {};

  tf2::BufferCore m_tf_buffer {};
  tf2_ros::TransformListener m_tf_listener;
};
}  // namespace tf2_detector
#endif  // TF2_DETECTOR_OBSTACLEDETECTORIMPROVED_H // NOLINT
