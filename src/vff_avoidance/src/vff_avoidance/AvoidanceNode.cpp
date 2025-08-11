#include "../../include/vff_avoidance/AvoidanceNode.h"

// #include "vff_avoidance/AvoidanceNode.h"

#include <algorithm>
#include <cmath>
#include <rclcpp/qos.hpp>
#include <string>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;

namespace vff_avoidance {

/*VFFVectors& VFFVectors::operator=(const VFFVectors& other) {*/
/*  this->attractive = other.attractive;*/
/*  this->repulsive = other.repulsive;*/
/*  this->result = other.result;*/
/*  return *this;*/
/*}*/

/*VFFVectors& VFFVectors::operator=(const VFFVectors& other) noexcept =
 * default;*/
/*VFFVectors& VFFVectors::operator=(VFFVectors&& other) noexcept = default;*/
/*VFFVectors::VFFVectors(const VFFVectors& other) = default;*/

AvoidanceNode::AvoidanceNode()
    : Node(NODE_NAME),
      mTimer{create_wall_timer(50ms, [this]() { control_cycle(); })},
      mVelPub{create_publisher<geometry_msgs::msg::Twist>(VEL_PUB_TOPIC, 100)},
      mVffDebugPub{
          create_publisher<visualization_msgs::msg::MarkerArray>(
              VFFDEBUG_PUB_TOPIC, 100),
      },
      mTFBuffer{},
      mTFListener{mTFBuffer} {
  declare_parameter("obstacle_frames", std::vector<std::string>());

  obstacleFrames = get_parameter("obstacle_frames").as_string_array();
}

void AvoidanceNode::scan_callback(const size_t index,
                                  sensor_msgs::msg::LaserScan::UniquePtr msg) {
  // mLastScan[index] = std::move(msg);
}

void AvoidanceNode::control_cycle() {

  const VFFVectors &vff{get_vff()};

  const auto &v{vff.result};
  const double angle{atan2(v[1], v[0])};
  const double module{sqrt(v[0] * v[0] + v[1] * v[1])};

  geometry_msgs::msg::Twist vel{};
  vel.linear.x = std::clamp(module, 0.0, 0.3);
  vel.angular.z = std::clamp(angle, -0.5, 0.5);

  mVelPub->publish(vel);

  if (mVffDebugPub->get_subscription_count() > 0) {
    mVffDebugPub->publish(get_debug_vff(vff));
  }
}

auto AvoidanceNode::get_debug_vff(const VFFVectors &vff_vectors)
    -> visualization_msgs::msg::MarkerArray {
  visualization_msgs::msg::MarkerArray marker_array{};

  marker_array.markers.push_back(make_marker(vff_vectors.attractive, BLUE));
  marker_array.markers.push_back(make_marker(vff_vectors.repulsive, RED));
  marker_array.markers.push_back(make_marker(vff_vectors.result, GREEN));

  return marker_array;
}

visualization_msgs::msg::Marker AvoidanceNode::make_marker(
    const std::vector<float> &vector, VFFColor vff_color) {
  visualization_msgs::msg::Marker marker{};

  marker.header.frame_id = "base_footprint";
  marker.header.stamp = now();
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.id = visualization_msgs::msg::Marker::ADD;

  geometry_msgs::msg::Point start{};
  start.x = 0.0;
  start.y = 0.0;

  geometry_msgs::msg::Point end{};
  end.x = vector[0];
  end.y = vector[1];
  marker.points = {start, end};

  marker.scale.x = 0.05;
  marker.scale.y = 0.1;

  switch (vff_color) {
    case RED:
      marker.id = 0;
      marker.color.r = 1.0;
      break;
    case GREEN:
      marker.id = 1;
      marker.color.g = 1.0;
      break;
    case BLUE:
      marker.id = 2;
      marker.color.b = 1.0;
      break;
    case NUM_COLORS:
      break;
  }
  marker.color.a = 1.0;

  return marker;
}

auto AvoidanceNode::get_vff() const -> VFFVectors {
  VFFVectors vff_vector{};

  vff_vector.attractive = {MAX_OBSTACLE_DISTANCE_METER, 0.0};
  vff_vector.repulsive = {0.0, 0.0};
  vff_vector.result = {1.0, 0.0};

  tf2::Vector3 obstacleSum{0, 0, 0};

  for (const auto &obstacleFrame : obstacleFrames) {
    try {
      const auto obstacle{mTFBuffer
                              .lookupTransform("base_footprint", obstacleFrame,
                                               tf2::TimePointZero)
                              .transform.translation};
      obstacleSum.setX(obstacleSum.x() + obstacle.x);
      obstacleSum.setY(obstacleSum.y() + obstacle.y);
      obstacleSum.setZ(obstacleSum.z() + obstacle.z);

      tf2::Vector3 obstacleVec {obstacle.x, obstacle.y, obstacle.z};

      const auto distToObstacle {obstacleVec.length()};
  
      if (distToObstacle < MAX_OBSTACLE_DISTANCE_METER) {
        const auto newLen {MAX_OBSTACLE_DISTANCE_METER - distToObstacle};

        obstacleVec.setX(obstacleVec.x() / distToObstacle * newLen);
        obstacleVec.setY(obstacleVec.y() / distToObstacle * newLen);
        obstacleVec.setZ(obstacleVec.z() / distToObstacle * newLen);

        vff_vector.repulsive[0] -= obstacleVec.x();
        vff_vector.repulsive[1] -= obstacleVec.y();

        RCLCPP_INFO(get_logger(), "Repulsive vector: {x: %s, y: %s }", std::to_string(vff_vector.repulsive[0]).data(), std::to_string(vff_vector.repulsive[1]).data());
      }


    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(get_logger(), "Obstacle transform not found: %s", ex.what());
    }
  }

  // const auto min_idx = std::min_element(scan->ranges.begin(),
  // scan->ranges.end())
  // - scan->ranges.begin();
  //
  // const float distanceMin {scan->ranges[min_idx]};
  //
  // if (distanceMin < MAX_OBSTACLE_DISTANCE_METER) {
  //   const float angle {scan->angle_min + scan->angle_increment
  //
  //   vff_vector.repulsive[0] = cos(oposite_angle) * compementary_dist;
  //   vff_vector.repulsive[1] = sin(oposite_angle) * compementary_dist;
  // }
  vff_vector.result[0] = (vff_vector.repulsive[0] + vff_vector.attractive[0]);
  vff_vector.result[1] = (vff_vector.repulsive[1] + vff_vector.attractive[1]);

  return vff_vector;
}
}  // namespace vff_avoidance
