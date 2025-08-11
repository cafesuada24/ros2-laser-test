#include "tf2_detector/ObstacleMonitorNode.h"
// #include "../../include/tf2_detector/ObstacleMonitorNode.h"

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2/exceptions.hpp>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace tf2_detector {

using namespace std::chrono_literals;

ObstacleMonitorNode::ObstacleMonitorNode()
    : Node(NODE_NAME),
      m_timer{create_wall_timer(500ms, [this]() { control_cycle(); })},
      m_tf_buffer{},
      m_tf_listener{m_tf_buffer},
      m_marker_pub{create_publisher<visualization_msgs::msg::Marker>(
          "obstacle_marker", 1)} {
  declare_parameter("obstacle_frames", std::vector<std::string>());
  
  obstacleFrames = get_parameter("obstacle_frames").as_string_array();
}

void ObstacleMonitorNode::control_cycle() {

  for (const auto& obs : obstacleFrames ) {
    publishObstacleArrow(obs);
  }
}

void ObstacleMonitorNode::publishObstacleArrow(const std::string obstacleName) const {
  geometry_msgs::msg::TransformStamped robot2obstacle;

  try {
    robot2obstacle = m_tf_buffer.lookupTransform(
        "odom", obstacleName, tf2::TimePointZero);
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(get_logger(), "Obstacle transform not found: %s", ex.what());
    return;
  }

  const double x{robot2obstacle.transform.translation.x};
  const double y{robot2obstacle.transform.translation.y};
  const double z{robot2obstacle.transform.translation.z};
  const double theta{atan2(y, x)};

  RCLCPP_INFO(get_logger(),
              "Obstacle detected at (%lf m, %lf m, %lf m) = %lf rads", x, y, z,
              theta);

  visualization_msgs::msg::Marker obstacle_arrow{};

  obstacle_arrow.header.frame_id = "sonar_" + std::to_string(static_cast<int>(obstacleName.back()) - 48 + 1);
  obstacle_arrow.header.stamp = now();
  obstacle_arrow.type = visualization_msgs::msg::Marker::ARROW;
  obstacle_arrow.action = visualization_msgs::msg::Marker::ADD;
  obstacle_arrow.lifetime = rclcpp::Duration(1s);

  geometry_msgs::msg::Point start{};
  start.x = 0.0;
  start.y = 0.0;
  start.z = 0.0;
  geometry_msgs::msg::Point end{};
  end.x = x;
  end.y = y;
  end.z = z;
  obstacle_arrow.points = {start, end};

  obstacle_arrow.color.r = 1.0;
  obstacle_arrow.color.g = 0.0;
  obstacle_arrow.color.b = 0.0;
  obstacle_arrow.color.a = 1.0;

  obstacle_arrow.scale.x = 0.02;
  obstacle_arrow.scale.y = 0.1;
  obstacle_arrow.scale.z = 0.1;

  m_marker_pub->publish(obstacle_arrow);
}

}  // namespace tf2_detector
