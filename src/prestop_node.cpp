#include "prestop/prestop_node.hpp"

#include <algorithm>
#include <chrono>
#include <functional>

using namespace std::chrono_literals;

namespace prestop
{

PrestopNode::PrestopNode()
: Node("prestop_node")
{
  wait_duration_ = declare_parameter<double>("wait_duration", 3.0);
  clear_reset_duration_ = declare_parameter<double>("clear_reset_duration", 1.0);
  cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.5);
  publish_rate_ = declare_parameter<double>("publish_rate", 20.0);
  slowdown_enabled_ = declare_parameter<bool>("slowdown_enabled", true);
  slowdown_ratio_ = declare_parameter<double>("slowdown_ratio", 0.3);
  slowdown_ratio_ = std::clamp(slowdown_ratio_, 0.0, 1.0);

  const std::vector<std::string> default_target_polygons;
  const auto target_polygons =
    declare_parameter<std::vector<std::string>>("target_polygons", default_target_polygons);
  target_polygons_.insert(target_polygons.begin(), target_polygons.end());

  const std::vector<std::string> default_slowdown_polygons;
  const auto slowdown_polygons =
    declare_parameter<std::vector<std::string>>("slowdown_polygons", default_slowdown_polygons);
  slowdown_polygons_.insert(slowdown_polygons.begin(), slowdown_polygons.end());
  if (slowdown_polygons_.empty()) {
    slowdown_polygons_ = target_polygons_;
  }

  state_enter_time_ = now();

  collision_state_sub_ = create_subscription<nav2_msgs::msg::CollisionDetectorState>(
    "collision_detector_state",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&PrestopNode::collisionStateCallback, this, std::placeholders::_1));

  cmd_vel_raw_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel_raw",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&PrestopNode::cmdVelRawCallback, this, std::placeholders::_1));

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::SystemDefaultsQoS());
  state_pub_ = create_publisher<std_msgs::msg::String>("prestop_state", 10);

  const auto period = std::chrono::duration<double>(1.0 / std::max(publish_rate_, 1.0));
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&PrestopNode::timerCallback, this));

  RCLCPP_INFO(
    get_logger(),
    "prestop_node started: wait_duration=%.2f sec, clear_reset_duration=%.2f sec",
    wait_duration_,
    clear_reset_duration_);
}

void PrestopNode::collisionStateCallback(
  const nav2_msgs::msg::CollisionDetectorState::SharedPtr msg)
{
  obstacle_detected_ = selectedDetectionPresent(*msg);
  slowdown_detected_ = slowdown_enabled_ && detectionPresent(*msg, slowdown_polygons_);
}

void PrestopNode::cmdVelRawCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  last_cmd_vel_raw_ = *msg;
  last_cmd_vel_raw_time_ = now();
  has_cmd_vel_raw_ = true;
}

void PrestopNode::timerCallback()
{
  const rclcpp::Time current_time = now();
  updateState(current_time);
  publishCmdVel(current_time);
  publishState();
}

bool PrestopNode::selectedDetectionPresent(
  const nav2_msgs::msg::CollisionDetectorState & msg) const
{
  return detectionPresent(msg, target_polygons_);
}

bool PrestopNode::detectionPresent(
  const nav2_msgs::msg::CollisionDetectorState & msg,
  const std::set<std::string> & polygon_names) const
{
  const size_t count = std::min(msg.polygons.size(), msg.detections.size());
  for (size_t i = 0; i < count; ++i) {
    if (!msg.detections[i]) {
      continue;
    }
    if (polygon_names.empty() || polygon_names.count(msg.polygons[i]) > 0) {
      return true;
    }
  }
  return false;
}

void PrestopNode::updateState(const rclcpp::Time & current_time)
{
  if (obstacle_detected_) {
    has_clear_since_ = false;
  } else if (!has_clear_since_) {
    clear_since_ = current_time;
    has_clear_since_ = true;
  }

  switch (state_) {
    case GateState::NORMAL:
      if (obstacle_detected_) {
        setState(GateState::STOPPED_WAITING, current_time);
      }
      break;

    case GateState::STOPPED_WAITING:
      if (elapsedSince(state_enter_time_, current_time) >= wait_duration_) {
        setState(GateState::PASS_THROUGH_AFTER_WAIT, current_time);
      }
      break;

    case GateState::PASS_THROUGH_AFTER_WAIT:
      if (!obstacle_detected_ && has_clear_since_ &&
        elapsedSince(clear_since_, current_time) >= clear_reset_duration_)
      {
        setState(GateState::NORMAL, current_time);
      }
      break;
  }
}

void PrestopNode::publishCmdVel(const rclcpp::Time & current_time)
{
  if (state_ == GateState::STOPPED_WAITING) {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    return;
  }

  geometry_msgs::msg::Twist cmd;
  if (has_cmd_vel_raw_ && elapsedSince(last_cmd_vel_raw_time_, current_time) <= cmd_timeout_) {
    cmd = last_cmd_vel_raw_;
  }

  if (slowdown_detected_) {
    scaleTwist(cmd, slowdown_ratio_);
  }

  cmd_vel_pub_->publish(cmd);
}

void PrestopNode::publishState()
{
  std_msgs::msg::String msg;
  msg.data = stateToString(state_);
  state_pub_->publish(msg);
}

void PrestopNode::setState(
  const GateState new_state,
  const rclcpp::Time & current_time)
{
  if (state_ == new_state) {
    return;
  }

  RCLCPP_INFO(
    get_logger(),
    "State changed: %s -> %s",
    stateToString(state_).c_str(),
    stateToString(new_state).c_str());

  state_ = new_state;
  state_enter_time_ = current_time;
}

void PrestopNode::scaleTwist(geometry_msgs::msg::Twist & twist, const double ratio)
{
  twist.linear.x *= ratio;
  twist.linear.y *= ratio;
  twist.linear.z *= ratio;
  twist.angular.x *= ratio;
  twist.angular.y *= ratio;
  twist.angular.z *= ratio;
}

double PrestopNode::elapsedSince(
  const rclcpp::Time & start_time,
  const rclcpp::Time & current_time)
{
  return (current_time - start_time).seconds();
}

std::string PrestopNode::stateToString(const GateState state)
{
  switch (state) {
    case GateState::NORMAL:
      return "NORMAL";
    case GateState::STOPPED_WAITING:
      return "STOPPED_WAITING";
    case GateState::PASS_THROUGH_AFTER_WAIT:
      return "PASS_THROUGH_AFTER_WAIT";
  }
  return "UNKNOWN";
}

}  // namespace prestop

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<prestop::PrestopNode>());
  rclcpp::shutdown();
  return 0;
}
