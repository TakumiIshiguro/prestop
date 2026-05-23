#include "prestop/prestop_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <limits>
#include <stdexcept>

namespace prestop
{

using namespace std::chrono_literals;

PrestopNode::PrestopNode()
: Node("prestop_node")
{
  cmd_vel_in_topic_ = declare_parameter<std::string>("cmd_vel_in_topic", "/cmd_vel_raw");
  cmd_vel_out_topic_ = declare_parameter<std::string>("cmd_vel_out_topic", "/cmd_vel");
  scan_in_topic_ = declare_parameter<std::string>("scan_in_topic", "/scan_raw");
  scan_out_topic_ = declare_parameter<std::string>("scan_out_topic", "/scan");
  empty_scan_mode_ = declare_parameter<std::string>("empty_scan_mode", "inf");
  base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_link");
  visualize_stop_zone_ = declare_parameter<bool>("visualize_stop_zone", true);
  stop_zone_polygon_topic_ =
    declare_parameter<std::string>("stop_zone_polygon_topic", "/prestop/stop_zone_polygon");
  stop_duration_ = declare_parameter<double>("stop_duration", 3.0);

  stop_zone_enabled_ = declare_parameter<bool>("polygons.stop_zone.enabled", true);
  stop_zone_action_type_ =
    declare_parameter<std::string>("polygons.stop_zone.action_type", "stop");
  const auto points_string = declare_parameter<std::string>(
    "polygons.stop_zone.points",
    "[[0.20, 0.35], [0.80, 0.35], [0.80, -0.35], [0.20, -0.35]]");

  stop_zone_ = parsePolygonPoints(points_string);
  if (stop_zone_.size() < 3) {
    throw std::runtime_error("polygons.stop_zone.points must contain at least 3 points");
  }
  if (stop_zone_action_type_ != "stop") {
    throw std::runtime_error("polygons.stop_zone.action_type must be \"stop\"");
  }
  if (empty_scan_mode_ != "inf") {
    throw std::runtime_error("empty_scan_mode currently supports only \"inf\"");
  }
  if (stop_duration_ < 0.0) {
    throw std::runtime_error("stop_duration must be non-negative");
  }

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_out_topic_, 10);
  scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
    scan_out_topic_, rclcpp::SensorDataQoS());
  if (visualize_stop_zone_) {
    stop_zone_polygon_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>(
      stop_zone_polygon_topic_, rclcpp::QoS(1).transient_local());
    polygon_timer_ = create_wall_timer(1s, std::bind(&PrestopNode::publishStopZonePolygon, this));
    publishStopZonePolygon();
  }

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_in_topic_,
    10,
    std::bind(&PrestopNode::cmdVelCallback, this, std::placeholders::_1));
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_in_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&PrestopNode::scanCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "prestop_node started: cmd_vel %s -> %s, scan %s -> %s, initial_state=%s",
    cmd_vel_in_topic_.c_str(),
    cmd_vel_out_topic_.c_str(),
    scan_in_topic_.c_str(),
    scan_out_topic_.c_str(),
    stateToString(state_).c_str());
}

void PrestopNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (!has_scan_ || state_ == FilterState::STOP) {
    publishZeroCmdVel();
    return;
  }

  cmd_vel_pub_->publish(*msg);
}

void PrestopNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  has_scan_ = true;

  const auto previous_state = state_;
  const bool obstacle_detected = stop_zone_enabled_ && scanHasObstacleInStopZone(*msg);
  const rclcpp::Time current_time = now();

  if (!obstacle_detected) {
    waiting_for_clear_ = false;
    state_ = FilterState::CLEAR;
  } else if (waiting_for_clear_) {
    state_ = FilterState::CLEAR;
  } else if (previous_state != FilterState::STOP) {
    stop_start_time_ = current_time;
    state_ = FilterState::STOP;
  } else if ((current_time - stop_start_time_).seconds() >= stop_duration_) {
    waiting_for_clear_ = true;
    state_ = FilterState::CLEAR;
  } else {
    state_ = FilterState::STOP;
  }

  if (state_ != previous_state) {
    RCLCPP_INFO(
      get_logger(),
      "State changed: %s -> %s",
      stateToString(previous_state).c_str(),
      stateToString(state_).c_str());
  }

  if (obstacle_detected) {
    scan_pub_->publish(*msg);
  } else {
    scan_pub_->publish(makeEmptyScan(*msg));
  }

  if (state_ == FilterState::STOP) {
    publishZeroCmdVel();
  }
}

bool PrestopNode::scanHasObstacleInStopZone(const sensor_msgs::msg::LaserScan & scan) const
{
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const float range = scan.ranges[i];
    if (!std::isfinite(range)) {
      continue;
    }
    if (range < scan.range_min || range > scan.range_max) {
      continue;
    }

    const double angle = static_cast<double>(scan.angle_min) +
      static_cast<double>(i) * static_cast<double>(scan.angle_increment);
    const Point2D point{
      static_cast<double>(range) * std::cos(angle),
      static_cast<double>(range) * std::sin(angle)};

    if (pointInPolygon(point)) {
      return true;
    }
  }
  return false;
}

bool PrestopNode::pointInPolygon(const Point2D & point) const
{
  bool inside = false;
  size_t j = stop_zone_.size() - 1;
  for (size_t i = 0; i < stop_zone_.size(); ++i) {
    const Point2D & pi = stop_zone_[i];
    const Point2D & pj = stop_zone_[j];
    const bool intersects = ((pi.y > point.y) != (pj.y > point.y)) &&
      (point.x < (pj.x - pi.x) * (point.y - pi.y) / (pj.y - pi.y) + pi.x);
    if (intersects) {
      inside = !inside;
    }
    j = i;
  }
  return inside;
}

sensor_msgs::msg::LaserScan PrestopNode::makeEmptyScan(
  const sensor_msgs::msg::LaserScan & scan) const
{
  sensor_msgs::msg::LaserScan empty_scan = scan;
  std::fill(
    empty_scan.ranges.begin(),
    empty_scan.ranges.end(),
    std::numeric_limits<float>::infinity());
  empty_scan.intensities.clear();
  return empty_scan;
}

void PrestopNode::publishZeroCmdVel()
{
  cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
}

void PrestopNode::publishStopZonePolygon()
{
  if (!stop_zone_polygon_pub_) {
    return;
  }

  geometry_msgs::msg::PolygonStamped polygon;
  polygon.header.stamp = now();
  polygon.header.frame_id = base_frame_id_;
  polygon.polygon.points.reserve(stop_zone_.size());
  for (const auto & point : stop_zone_) {
    geometry_msgs::msg::Point32 polygon_point;
    polygon_point.x = static_cast<float>(point.x);
    polygon_point.y = static_cast<float>(point.y);
    polygon_point.z = 0.0f;
    polygon.polygon.points.push_back(polygon_point);
  }

  stop_zone_polygon_pub_->publish(polygon);
}

std::vector<Point2D> PrestopNode::parsePolygonPoints(const std::string & points_string)
{
  std::vector<double> values;
  const char * cursor = points_string.c_str();
  char * end = nullptr;

  while (*cursor != '\0') {
    const double value = std::strtod(cursor, &end);
    if (end != cursor) {
      values.push_back(value);
      cursor = end;
      continue;
    }
    ++cursor;
  }

  if (values.size() % 2 != 0) {
    throw std::runtime_error("polygons.stop_zone.points must contain x/y pairs");
  }

  std::vector<Point2D> points;
  points.reserve(values.size() / 2);
  for (size_t i = 0; i < values.size(); i += 2) {
    points.push_back(Point2D{values[i], values[i + 1]});
  }
  return points;
}

std::string PrestopNode::stateToString(const FilterState state)
{
  switch (state) {
    case FilterState::CLEAR:
      return "CLEAR";
    case FilterState::STOP:
      return "STOP";
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
