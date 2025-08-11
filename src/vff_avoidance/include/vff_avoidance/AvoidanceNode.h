#ifndef VFF_AVOIDANCE_AVOIDANCENODE_H_
#define VFF_AVOIDANCE_AVOIDANCENODE_H_

#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <rclcpp/clock.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2/buffer_core.hpp>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace vff_avoidance {

using namespace std::chrono_literals;

struct VFFVectors {
  std::vector<float> attractive{0, 0, 0};
  std::vector<float> repulsive{0, 0, 0};
  std::vector<float> result{0, 0, 0};
};

enum VFFColor { RED, GREEN, BLUE, NUM_COLORS };

class AvoidanceNode : public rclcpp::Node {
 public:
  AvoidanceNode();

 private:
  static inline const std::string NODE_NAME{"avoidance_vff"};

  rclcpp::TimerBase::SharedPtr mTimer{};

  static constexpr float MAX_OBSTACLE_DISTANCE_METER{1.0f};
  static constexpr std::chrono::seconds SCAN_TIMEOUT{1s};

  void control_cycle();
  auto get_vff() const -> VFFVectors;

  const std::string VEL_PUB_TOPIC{"output_vel"};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr mVelPub{};

  const std::string SCAN_SUB_TOPIC{"ultrasonic_scan"};
  // std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr>
  //     mScanSub;
  
  std::vector<std::string> obstacleFrames;

  std::vector<sensor_msgs::msg::LaserScan::UniquePtr> mLastScan{};
  void scan_callback(const size_t index,
                     sensor_msgs::msg::LaserScan::UniquePtr msg);

  auto get_debug_vff(const VFFVectors& vff_vectors)
      -> visualization_msgs::msg::MarkerArray;
  auto make_marker(const std::vector<float>& vector, VFFColor vff_color)
      -> visualization_msgs::msg::Marker;

  const std::string VFFDEBUG_PUB_TOPIC{"vff_debug"};
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      mVffDebugPub{};


  tf2::BufferCore mTFBuffer;
  tf2_ros::TransformListener mTFListener;
};
}  // namespace vff_avoidance

#endif  // VFF_AVOIDANCE__AVOIDANCENODE_H
