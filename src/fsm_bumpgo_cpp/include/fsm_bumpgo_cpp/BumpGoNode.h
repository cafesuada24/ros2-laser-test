#ifndef FSM_BUMPGO_CPP__BUMPGONODE_H  // NOLINT
#define FSM_BUMPGO_CPP__BUMPGONODE_H

#include <memory>
/*#include <rclcpp/parameter_event_handler.hpp>*/

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
namespace fsm_bumpgo_cpp {
using namespace std::chrono_literals;  // NOLINT

class BumpGoNode : public rclcpp::Node {
 public:
  BumpGoNode();

 private:
  void scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg);
  void key_input_callback(geometry_msgs::msg::Twist::UniquePtr msg);
  inline void control_cycle();
  void control_auto();
  void control_soft();
  void control_hard();
  inline void stop_moving() const;

  static constexpr int FORWARD{0};
  static constexpr int BACK{1};
  static constexpr int TURN{2};
  static constexpr int STOP{3};

  int m_state{};              // current state
  rclcpp::Time m_state_ts{};  // transition time

  void go_state(const int new_state);
  bool check_fw2stop() const;
  bool check_fw2back() const;
  bool check_back2turn() const;
  bool check_turn2fw() const;
  bool check_stop2fw() const;

  const rclcpp::Duration SCAN_TIMEOUT{1s};
  const rclcpp::Duration BACKING_TIME{2s};
  const rclcpp::Duration TURNING_TIME{1s};

  static constexpr float OBSTACLE_DISTANCE{1.0f};
  static constexpr float SPEED_LINEAR{0.5f};
  static constexpr float SPEED_ANGULAR{0.5f};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_vel_pub{};
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_scan_sub{};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_key_sub{};
  rclcpp::TimerBase::SharedPtr m_timer{};

  sensor_msgs::msg::LaserScan::UniquePtr m_last_scan{nullptr};
  geometry_msgs::msg::Twist::UniquePtr m_last_key{nullptr};

  static constexpr int MODE_AUTO{1};
  static constexpr int MODE_SOFT_CTL{3};
  static constexpr int MODE_HARD_CTL{2};

  /*std::shared_ptr<rclcpp::ParameterEventHandler> m_param_subscriber;*/
  std::shared_ptr<rclcpp::ParameterCallbackHandle> m_param_cb;

  int ctl_mode {};
};

};  // namespace fsm_bumpgo_cpp

#endif  // FSM_BUMPGO_CPP__BUMPGONODE_H // NOLINT
