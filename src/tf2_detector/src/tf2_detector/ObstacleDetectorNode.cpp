#include "tf2_detector/ObstacleDetectorNode.h"
// #include "../../include/tf2_detector/ObstacleDetectorNode.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>

#include <algorithm>
#include <memory>
#include <rclcpp/qos.hpp>
#include <string>
#include <tf2/LinearMath/Transform.hpp>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace tf2_detector {

using namespace std::chrono_literals;

using std::placeholders::_1;

ObstacleDetectorNode::ObstacleDetectorNode()
    : Node(NODE_NAME),
      m_tf_broadcaster{
          std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this)},
      m_tf_buffer{},
      m_tf_listener{m_tf_buffer} {
  this->declare_parameter("input_scans", std::vector<std::string>());
  this->declare_parameter("scanner_frames", std::vector<std::string>());

  inputScans = get_parameter("input_scans").as_string_array();
  scannerFrames = get_parameter("scanner_frames").as_string_array();

  assert(inputScans.size() == scannerFrames.size());

  const auto numScanner{inputScans.size()};
  mScanSub.reserve(numScanner);

  for (size_t index{0}; index < numScanner; ++index) {
    mScanSub.push_back(create_subscription<sensor_msgs::msg::LaserScan>(
        inputScans[index], rclcpp::SensorDataQoS(),
        [this, index](sensor_msgs::msg::LaserScan::UniquePtr msg) {
          scan_callback(index, std::move(msg));
        }));
    RCLCPP_INFO(get_logger(), "Registered to %s", inputScans[index].data());
  }
}

void ObstacleDetectorNode::scan_callback(
    const size_t index, sensor_msgs::msg::LaserScan::UniquePtr msg) {
  const double dist{*std::min_element(msg->ranges.begin(), msg->ranges.end())};

  if (std::isinf(dist)) {
    return;
  }

  tf2::Transform laser2object{};
  laser2object.setOrigin(tf2::Vector3(dist, 0.0, 0.0));
  laser2object.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

  geometry_msgs::msg::TransformStamped odom2laser_msg{};
  tf2::Stamped<tf2::Transform> odom2laser{};
  try {
    odom2laser_msg = m_tf_buffer.lookupTransform(
        "odom", scannerFrames[index],
        tf2::timeFromSec(rclcpp::Time(msg->header.stamp).seconds() - 0.3));
    tf2::fromMsg(odom2laser_msg, odom2laser);
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(get_logger(), "Transform not found: %s", ex.what());
    return;
  }

  const tf2::Transform odom2object{odom2laser * laser2object};

  geometry_msgs::msg::TransformStamped odom2object_msg{};
  odom2object_msg.transform = tf2::toMsg(odom2object);

  odom2object_msg.header.stamp = msg->header.stamp;
  odom2object_msg.header.frame_id = "odom";
  odom2object_msg.child_frame_id = "detected_obstacle_" + std::to_string(index);

  m_tf_broadcaster->sendTransform(odom2object_msg);

  RCLCPP_INFO(get_logger(), "%s", ("Broadcasted to detected_obstacle_" + std::to_string(index)).data());
}
}  // namespace tf2_detector
