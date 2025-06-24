#include "fsm_bumpgo_cpp/BumpGoNode.h"
/*#include "../../include/fsm_bumpgo_cpp/BumpGoNode.h"*/

#include <rclcpp/logging.hpp>
#include <rclcpp/parameter.hpp>
#include <utility>

#include "geometry_msgs/msg/twist.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace fsm_bumpgo_cpp {

using namespace std::chrono_literals;
using std::placeholders::_1;

BumpGoNode::BumpGoNode()
    : Node("bump_go"),
      m_state{FORWARD},
      m_state_ts{now()},
      m_vel_pub{create_publisher<geometry_msgs::msg::Twist>("output_vel", 10)},
      m_scan_sub{create_subscription<sensor_msgs::msg::LaserScan>(  // NOLINT
          "input_scan", rclcpp::SensorDataQoS(),
          [this](sensor_msgs::msg::LaserScan::UniquePtr msg) {
            scan_callback(std::move(msg));
          })},                                                   // NOLINT
      m_key_sub{create_subscription<geometry_msgs::msg::Twist>(  // NOLINT
          "input_key", 10,
          [this](geometry_msgs::msg::Twist::UniquePtr msg) {
            key_input_callback(std::move(msg));
          })},
      m_timer{create_wall_timer(50ms, [this]() { control_cycle(); })},
      /*m_param_subscriber{std::make_shared<rclcpp::ParameterEventHandler>(this)},*/
      ctl_mode{MODE_AUTO} {
  declare_parameter("control_mode", MODE_AUTO);
  /*auto cb = [this](const std::vector<rclcpp::Parameter>& params)*/
  /*    -> rcl_interfaces::msg::SetParametersResult {*/
  /*  rcl_interfaces::msg::SetParametersResult result{};*/
  /*  result.successful = true;*/
  /*  const auto p{params.front()};*/
  /*  const long val{p.as_int()};*/
  /*  if (val != MODE_AUTO && val != MODE_SOFT_CTL && val != MODE_HARD_CTL) {*/
  /*    RCLCPP_INFO(get_logger(),*/
  /*                "invalid mode value, expected: 1 (Auto), 2 (Hard Control), 3 "*/
  /*                "(Soft Control)");*/
  /*    set_parameter(rclcpp::Parameter("control_mode", ctl_mode));*/
  /*    result.successful = false;*/
  /*    return result;*/
  /*  }*/
  /*  ctl_mode = p.as_int();*/
  /*  stop_moving();*/
  /*  RCLCPP_INFO(get_logger(), "control mode is set to %d", ctl_mode);*/
  /*  return result;*/
  /*};*/

  /*m_param_cb = m_param_subscriber->add_parameter_callback("control_mode",
   * cb);*/
  /*this->add_on_set_parameters_callback(cb);*/
}

/*int BumpGoNode::get_mode() const {*/
/*  return this->get_parameter("control_mode").as_int();*/
/*}*/

inline void BumpGoNode::stop_moving() const {
  geometry_msgs::msg::Twist msg{};
  m_vel_pub->publish(msg);
}

void BumpGoNode::scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg) {
  m_last_scan = std::move(msg);
}

void BumpGoNode::key_input_callback(geometry_msgs::msg::Twist::UniquePtr msg) {
  m_last_key = std::move(msg);
}

void BumpGoNode::go_state(const int new_state) {
  m_state = new_state;
  m_state_ts = now();
}

void BumpGoNode::control_soft() {}

void BumpGoNode::control_hard() {
  if (this->get_parameter("control_mode").as_int() != MODE_HARD_CTL) {
    return;
  }
  if (m_last_key == nullptr) return;

  /*geometry_msgs::msg::Twist out_vel{};*/

  m_vel_pub->publish(*m_last_key);
  m_last_key = nullptr;
}
void BumpGoNode::control_auto() {
  if (this->get_parameter("control_mode").as_int() != MODE_AUTO) {
    return;
  }
  if (m_last_scan == nullptr) return;

  geometry_msgs::msg::Twist out_vel{};

  /*bool allMore = true;*/
  /*for (int i = 0; i < m_last_scan->ranges.size(); i++)*/
  /*{*/
  /*  if (m_last_scan->ranges[i] < 1.0)*/
  /*  {*/
  /*    allMore = false;*/
  /*    break;*/
  /*  }*/
  /*}*/
  /*if (allMore) //if all bigger than one*/
  /*{*/
  /*  out_vel.linear.x = SPEED_LINEAR;*/
  /*}*/
  /*else*/
  /*{*/
  /*  out_vel.angular.z = SPEED_ANGULAR;*/
  /*}*/

  /*const auto oldState{m_state};*/

  switch (m_state) {
    case FORWARD:
      out_vel.linear.x = SPEED_LINEAR;

      if (check_fw2stop()) {
        go_state(STOP);
      }
      if (check_fw2back()) {
        go_state(BACK);
      }
      break;
    case BACK:
      out_vel.linear.x = -SPEED_LINEAR;
      if (check_back2turn()) {
        go_state(TURN);
      }
      break;
    case TURN:
      out_vel.angular.z = -SPEED_ANGULAR;
      if (check_turn2fw()) {
        go_state(FORWARD);
      }
      break;
    case STOP:
      if (check_stop2fw()) {
        go_state(FORWARD);
      }
      break;
  }

  /*if (m_state != oldState) {*/
    m_vel_pub->publish(out_vel);
  /*}*/
}

inline void BumpGoNode::control_cycle() {
  switch (this->get_parameter("control_mode").as_int()) {
    case MODE_SOFT_CTL:
      control_soft();
      break;
    case MODE_HARD_CTL:
      control_hard();
      break;
    default:
      control_auto();
      break;
  }
}

bool BumpGoNode::check_fw2back() const {
  /*for (int i = 0; i < m_last_scan->ranges.size(); i++)*/
  /*{*/
  /*  if (m_last_scan->ranges[i] < 1.0)*/
  /*  {*/
  /*    return false;*/
  /*  }*/
  /*}*/
  /*return true;*/
  const auto pos{m_last_scan->ranges.size() >> 1};
  return m_last_scan->ranges[pos] < OBSTACLE_DISTANCE;
}

bool BumpGoNode::check_fw2stop() const {
  const auto elapsed{now() - rclcpp::Time(m_last_scan->header.stamp)};
  return elapsed > SCAN_TIMEOUT;
}

bool BumpGoNode::check_back2turn() const {
  const auto pos{m_last_scan->ranges.size() >> 1};
  return m_last_scan->ranges[pos] >= OBSTACLE_DISTANCE && (now() - m_state_ts) > BACKING_TIME;
  /*return (now() - m_state_ts) > BACKING_TIME;*/

  /*const auto pos{m_last_scan->ranges.size() >> 1};*/
  /*return m_last_scan->ranges[pos] > OBSTACLE_DISTANCE;*/
}

bool BumpGoNode::check_turn2fw() const {
  /*const auto pos{m_last_scan->ranges.size() >> 1};*/
  /*return m_last_scan->ranges[pos] < OBSTACLE_DISTANCE;*/
  return (now() - m_state_ts) > TURNING_TIME;
}

bool BumpGoNode::check_stop2fw() const {
  const auto elapsed{now() - rclcpp::Time(m_last_scan->header.stamp)};
  return elapsed < SCAN_TIMEOUT;
}
}  // namespace fsm_bumpgo_cpp
