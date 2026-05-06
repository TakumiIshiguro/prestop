#ifndef PRESTOP__PRESTOP_NODE_HPP_
#define PRESTOP__PRESTOP_NODE_HPP_

#include <set>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/msg/collision_detector_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace prestop
{

enum class GateState
{
  NORMAL,
  STOPPED_WAITING,
  PASS_THROUGH_AFTER_WAIT
};

class PrestopNode : public rclcpp::Node
{
public:
  PrestopNode();

private:
  void collisionStateCallback(const nav2_msgs::msg::CollisionDetectorState::SharedPtr msg);
  void cmdVelRawCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void timerCallback();

  bool selectedDetectionPresent(const nav2_msgs::msg::CollisionDetectorState & msg) const;
  void updateState(const rclcpp::Time & current_time);
  void publishCmdVel(const rclcpp::Time & current_time);
  void publishState();
  void setState(GateState new_state, const rclcpp::Time & current_time);

  static double elapsedSince(const rclcpp::Time & start_time, const rclcpp::Time & current_time);
  static std::string stateToString(GateState state);

  double wait_duration_{3.0};
  double clear_reset_duration_{1.0};
  double cmd_timeout_{0.5};
  double publish_rate_{20.0};
  std::set<std::string> target_polygons_;

  GateState state_{GateState::NORMAL};
  rclcpp::Time state_enter_time_;
  rclcpp::Time last_cmd_vel_raw_time_;
  rclcpp::Time clear_since_;

  geometry_msgs::msg::Twist last_cmd_vel_raw_;
  bool has_cmd_vel_raw_{false};
  bool obstacle_detected_{false};
  bool has_clear_since_{false};

  rclcpp::Subscription<nav2_msgs::msg::CollisionDetectorState>::SharedPtr collision_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_raw_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace prestop

#endif  // PRESTOP__PRESTOP_NODE_HPP_
