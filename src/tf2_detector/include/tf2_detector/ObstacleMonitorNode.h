#ifndef TF2_DETECTOR_OBSTACLEMONITORNODE_H
#define TF2_DETECTOR_OBSTACLEMONITORNODE_H

#include <tf2_ros/transform_listener.h>

#include <rclcpp/publisher.hpp>
#include <string>
#include <vector>

#include "rclcpp/node.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace tf2_detector {
class ObstacleMonitorNode : public rclcpp::Node {
 public:
  ObstacleMonitorNode();

 private:
  static inline const std::string NODE_NAME {"obstacle_monitor"};

  void control_cycle();

  void publishObstacleArrow(const std::string topicName) const;

  std::vector<std::string> obstacleFrames;

  rclcpp::TimerBase::SharedPtr m_timer;
  tf2::BufferCore m_tf_buffer;
  tf2_ros::TransformListener m_tf_listener;
  

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_marker_pub;
};
}  // namespace tf2_detector

#endif  // TF2_DETECTOR_OBSTACLEMONITORNODE_H // NOLINT
