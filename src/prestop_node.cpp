#include "prestop/prestop_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <limits>
#include <stdexcept>

#include "lifecycle_msgs/msg/state.hpp"

namespace prestop
{

using namespace std::chrono_literals;

PrestopNode::PrestopNode(const rclcpp::NodeOptions & options)
: Node("prestop_node", options)
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
  no_overtake_zone_polygon_topic_ = declare_parameter<std::string>(
    "no_overtake_zone_polygon_topic", "/prestop/no_overtake_zone_polygon");
  no_overtake_topic_ =
    declare_parameter<std::string>("no_overtake_topic", "/no_overtake_active");
  no_overtake_status_topic_ = declare_parameter<std::string>(
    "no_overtake_status_topic", "/prestop/no_overtake_active");
  state_topic_ = declare_parameter<std::string>("state_topic", "/prestop/state");
  local_costmap_clear_service_ = declare_parameter<std::string>(
    "local_costmap_clear_service", "/local_costmap/clear_entirely_local_costmap");
  global_costmap_clear_service_ = declare_parameter<std::string>(
    "global_costmap_clear_service", "/global_costmap/clear_entirely_global_costmap");
  local_costmap_lifecycle_service_ = declare_parameter<std::string>(
    "local_costmap_lifecycle_service", "/controller_server/get_state");
  global_costmap_lifecycle_service_ = declare_parameter<std::string>(
    "global_costmap_lifecycle_service", "/planner_server/get_state");
  stop_duration_ = declare_parameter<double>("stop_duration", 3.0);
  rearm_clear_duration_ = declare_parameter<double>("rearm_clear_duration", 1.0);
  min_obstacle_scan_duration_ =
    declare_parameter<double>("min_obstacle_scan_duration", 0.0);
  no_overtake_exit_delay_ = declare_parameter<double>("no_overtake_exit_delay", 1.0);
  min_points_in_polygon_ =
    declare_parameter<int>("noise_filter.min_points_in_polygon", 2);
  no_overtake_input_ = declare_parameter<bool>("no_overtake_default", true);
  no_overtake_active_ = no_overtake_input_;
  clear_costmaps_before_no_overtake_exit_ =
    declare_parameter<bool>("clear_costmaps_before_no_overtake_exit", true);

  stop_zone_enabled_ = declare_parameter<bool>("polygons.stop_zone.enabled", true);
  stop_zone_action_type_ =
    declare_parameter<std::string>("polygons.stop_zone.action_type", "stop");
  const auto stop_zone_points_string = declare_parameter<std::string>(
    "polygons.stop_zone.points",
    "[[0.20, 0.35], [0.80, 0.35], [0.80, -0.35], [0.20, -0.35]]");
  no_overtake_zone_enabled_ =
    declare_parameter<bool>("polygons.no_overtake_zone.enabled", true);
  no_overtake_zone_action_type_ = declare_parameter<std::string>(
    "polygons.no_overtake_zone.action_type", "no_overtake");
  const auto no_overtake_zone_points_string = declare_parameter<std::string>(
    "polygons.no_overtake_zone.points",
    "[[0.20, 0.35], [0.80, 0.35], [0.80, -0.35], [0.20, -0.35]]");

  stop_zone_ = parsePolygonPoints(stop_zone_points_string, "polygons.stop_zone.points");
  no_overtake_zone_ = parsePolygonPoints(
    no_overtake_zone_points_string, "polygons.no_overtake_zone.points");
  if (stop_zone_.size() < 3) {
    throw std::runtime_error("polygons.stop_zone.points must contain at least 3 points");
  }
  if (no_overtake_zone_.size() < 3) {
    throw std::runtime_error(
      "polygons.no_overtake_zone.points must contain at least 3 points");
  }
  if (stop_zone_action_type_ != "stop") {
    throw std::runtime_error("polygons.stop_zone.action_type must be \"stop\"");
  }
  if (no_overtake_zone_action_type_ != "no_overtake") {
    throw std::runtime_error(
      "polygons.no_overtake_zone.action_type must be \"no_overtake\"");
  }
  if (empty_scan_mode_ != "inf") {
    throw std::runtime_error("empty_scan_mode currently supports only \"inf\"");
  }
  if (stop_duration_ < 0.0) {
    throw std::runtime_error("stop_duration must be non-negative");
  }
  if (rearm_clear_duration_ < 0.0) {
    throw std::runtime_error("rearm_clear_duration must be non-negative");
  }
  if (min_obstacle_scan_duration_ < 0.0) {
    throw std::runtime_error("min_obstacle_scan_duration must be non-negative");
  }
  if (no_overtake_exit_delay_ < 0.0) {
    throw std::runtime_error("no_overtake_exit_delay must be non-negative");
  }
  if (min_points_in_polygon_ < 1) {
    throw std::runtime_error("noise_filter.min_points_in_polygon must be at least 1");
  }

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_out_topic_, 10);
  scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
    scan_out_topic_, rclcpp::SensorDataQoS());
  if (visualize_stop_zone_) {
    stop_zone_polygon_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>(
      stop_zone_polygon_topic_, rclcpp::QoS(1).transient_local());
    no_overtake_zone_polygon_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>(
      no_overtake_zone_polygon_topic_, rclcpp::QoS(1).transient_local());
    polygon_timer_ = create_wall_timer(100ms, std::bind(&PrestopNode::publishPolygons, this));
    publishPolygons();
  }
  no_overtake_status_pub_ = create_publisher<std_msgs::msg::Bool>(
    no_overtake_status_topic_, rclcpp::QoS(1).transient_local().reliable());
  state_pub_ = create_publisher<std_msgs::msg::String>(
    state_topic_, rclcpp::QoS(1).transient_local().reliable());
  local_costmap_clear_client_ = create_client<nav2_msgs::srv::ClearEntireCostmap>(
    local_costmap_clear_service_);
  global_costmap_clear_client_ = create_client<nav2_msgs::srv::ClearEntireCostmap>(
    global_costmap_clear_service_);
  local_costmap_lifecycle_client_ = create_client<lifecycle_msgs::srv::GetState>(
    local_costmap_lifecycle_service_);
  global_costmap_lifecycle_client_ = create_client<lifecycle_msgs::srv::GetState>(
    global_costmap_lifecycle_service_);

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_in_topic_,
    10,
    std::bind(&PrestopNode::cmdVelCallback, this, std::placeholders::_1));
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_in_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&PrestopNode::scanCallback, this, std::placeholders::_1));
  no_overtake_sub_ = create_subscription<std_msgs::msg::Bool>(
    no_overtake_topic_,
    rclcpp::QoS(10).reliable(),
    std::bind(&PrestopNode::noOvertakeCallback, this, std::placeholders::_1));

  publishState();
  publishNoOvertakeStatus();

  RCLCPP_INFO(
    get_logger(),
    "prestop_node started: cmd_vel %s -> %s, scan %s -> %s, no_overtake %s, "
    "initial_state=%s, no_overtake_default=%s, min_points_in_polygon=%d",
    cmd_vel_in_topic_.c_str(),
    cmd_vel_out_topic_.c_str(),
    scan_in_topic_.c_str(),
    scan_out_topic_.c_str(),
    no_overtake_topic_.c_str(),
    stateToString(state_).c_str(),
    no_overtake_active_ ? "true" : "false",
    min_points_in_polygon_);
}

void PrestopNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  const bool velocity_allowed = state_ == FilterState::CLEAR ||
    state_ == FilterState::WAITING_FOR_REARM;
  if (!has_scan_ || !velocity_allowed) {
    publishZeroCmdVel();
    return;
  }

  cmd_vel_pub_->publish(*msg);
}

void PrestopNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  has_scan_ = true;

  const auto previous_state = state_;
  const rclcpp::Time current_time = now();
  updateNoOvertakeState(current_time);
  const bool temporary_stop_obstacle = stop_zone_enabled_ &&
    scanHasObstacleInPolygon(*msg, stop_zone_);
  const bool no_overtake_obstacle = no_overtake_zone_enabled_ &&
    no_overtake_active_ && scanHasObstacleInPolygon(*msg, no_overtake_zone_);
  const bool obstacle_detected = temporary_stop_obstacle || no_overtake_obstacle;

  if (obstacle_detected) {
    has_obstacle_scan_ = true;
    last_obstacle_scan_time_ = current_time;
  }

  if (no_overtake_obstacle) {
    rearm_clear_timer_active_ = false;
    state_ = FilterState::NO_OVERTAKE_HOLD;
  } else {
    switch (previous_state) {
      case FilterState::CLEAR:
        if (temporary_stop_obstacle) {
          stop_start_time_ = current_time;
          rearm_clear_timer_active_ = false;
          state_ = FilterState::TIMED_STOP;
        }
        break;

      case FilterState::TIMED_STOP:
        if ((current_time - stop_start_time_).seconds() >= stop_duration_) {
          rearm_clear_timer_active_ = false;
          state_ = FilterState::WAITING_FOR_REARM;
        }
        break;

      case FilterState::WAITING_FOR_REARM:
        if (temporary_stop_obstacle) {
          rearm_clear_timer_active_ = false;
        } else if (!rearm_clear_timer_active_) {
          rearm_clear_start_time_ = current_time;
          rearm_clear_timer_active_ = true;
        } else if (
          (current_time - rearm_clear_start_time_).seconds() >= rearm_clear_duration_)
        {
          rearm_clear_timer_active_ = false;
          state_ = FilterState::CLEAR;
        }
        break;

      case FilterState::NO_OVERTAKE_HOLD:
        rearm_clear_timer_active_ = false;
        if (temporary_stop_obstacle) {
          stop_start_time_ = current_time;
          state_ = FilterState::TIMED_STOP;
        } else {
          state_ = FilterState::CLEAR;
        }
        break;
    }
  }

  if (state_ != previous_state) {
    RCLCPP_INFO(
      get_logger(),
      "State changed: %s -> %s",
      stateToString(previous_state).c_str(),
      stateToString(state_).c_str());
    if (previous_state == FilterState::NO_OVERTAKE_HOLD) {
      RCLCPP_INFO(
        get_logger(),
        "[NO_OVERTAKE_RELEASED] No-overtake hold released; current_state=%s",
        stateToString(state_).c_str());
    }
    publishState();
  }

  if (state_ == FilterState::NO_OVERTAKE_HOLD) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "[NO_OVERTAKE_STUCK] Obstacle detected in no-overtake zone; "
      "holding the robot until the obstacle clears");
  }

  const bool force_raw_scan = state_ == FilterState::TIMED_STOP ||
    state_ == FilterState::WAITING_FOR_REARM ||
    state_ == FilterState::NO_OVERTAKE_HOLD;
  const bool keep_obstacle_scan = force_raw_scan || obstacle_detected ||
    (has_obstacle_scan_ &&
    (current_time - last_obstacle_scan_time_).seconds() < min_obstacle_scan_duration_);

  if (keep_obstacle_scan) {
    scan_pub_->publish(*msg);
  } else {
    scan_pub_->publish(makeEmptyScan(*msg));
  }

  if (state_ == FilterState::TIMED_STOP || state_ == FilterState::NO_OVERTAKE_HOLD) {
    publishZeroCmdVel();
  }
}

void PrestopNode::noOvertakeCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    no_overtake_input_ = true;
    no_overtake_clear_pending_ = false;
    costmap_clear_in_progress_ = false;
    local_costmap_clear_done_ = false;
    global_costmap_clear_done_ = false;
    local_costmap_active_confirmed_ = false;
    global_costmap_active_confirmed_ = false;
    ++costmap_clear_generation_;
    if (!no_overtake_active_) {
      no_overtake_active_ = true;
      RCLCPP_INFO(get_logger(), "No-overtake zone entered");
      publishNoOvertakeStatus();
    }
    return;
  }

  no_overtake_input_ = false;
  if (no_overtake_active_ && !no_overtake_clear_pending_) {
    no_overtake_clear_start_time_ = now();
    no_overtake_clear_pending_ = true;
    RCLCPP_INFO(
      get_logger(),
      "No-overtake zone exit requested; waiting %.2f seconds",
      no_overtake_exit_delay_);
  }
}

void PrestopNode::updateNoOvertakeState(const rclcpp::Time & current_time)
{
  if (no_overtake_input_ || !no_overtake_clear_pending_) {
    return;
  }

  if ((current_time - no_overtake_clear_start_time_).seconds() < no_overtake_exit_delay_) {
    return;
  }

  if (clear_costmaps_before_no_overtake_exit_) {
    requestCostmapClear();
    return;
  }

  finishNoOvertakeExit();
}

void PrestopNode::requestCostmapClear()
{
  if (costmap_clear_in_progress_) {
    return;
  }

  if (!local_costmap_clear_client_->service_is_ready() ||
    !global_costmap_clear_client_->service_is_ready() ||
    !local_costmap_lifecycle_client_->service_is_ready() ||
    !global_costmap_lifecycle_client_->service_is_ready())
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "[NO_OVERTAKE_CLEAR_WAIT] Waiting for Nav2 costmap and lifecycle services; "
      "no-overtake mode remains active");
    return;
  }

  costmap_clear_in_progress_ = true;
  local_costmap_clear_done_ = false;
  global_costmap_clear_done_ = false;
  local_costmap_active_confirmed_ = false;
  global_costmap_active_confirmed_ = false;
  const uint64_t generation = ++costmap_clear_generation_;

  RCLCPP_INFO(
    get_logger(),
    "[NO_OVERTAKE_CLEAR_CHECK] Confirming local and global costmaps are active");

  auto local_state_request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
  local_costmap_lifecycle_client_->async_send_request(
    local_state_request,
    [this, generation](rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture future)
    {
      costmapLifecycleCallback(future, true, generation);
    });

  auto global_state_request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
  global_costmap_lifecycle_client_->async_send_request(
    global_state_request,
    [this, generation](rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture future)
    {
      costmapLifecycleCallback(future, false, generation);
    });
}

void PrestopNode::costmapLifecycleCallback(
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture future,
  const bool is_local,
  const uint64_t generation)
{
  if (generation != costmap_clear_generation_ || !costmap_clear_in_progress_ ||
    no_overtake_input_ || !no_overtake_clear_pending_)
  {
    return;
  }

  try {
    const auto response = future.get();
    if (response->current_state.id != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      costmap_clear_in_progress_ = false;
      local_costmap_active_confirmed_ = false;
      global_costmap_active_confirmed_ = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "[NO_OVERTAKE_CLEAR_WAIT] %s costmap is not active; "
        "no-overtake mode remains active",
        is_local ? "Local" : "Global");
      return;
    }
  } catch (const std::exception & exception) {
    costmap_clear_in_progress_ = false;
    local_costmap_active_confirmed_ = false;
    global_costmap_active_confirmed_ = false;
    RCLCPP_ERROR(
      get_logger(),
      "[NO_OVERTAKE_CLEAR_CHECK_FAILED] Failed to get %s costmap lifecycle state: %s; "
      "no-overtake mode remains active",
      is_local ? "local" : "global",
      exception.what());
    return;
  }

  if (is_local) {
    local_costmap_active_confirmed_ = true;
  } else {
    global_costmap_active_confirmed_ = true;
  }

  if (local_costmap_active_confirmed_ && global_costmap_active_confirmed_) {
    sendCostmapClearRequests(generation);
  }
}

void PrestopNode::sendCostmapClearRequests(const uint64_t generation)
{
  RCLCPP_INFO(
    get_logger(),
    "[NO_OVERTAKE_CLEARING] Clearing local and global costmaps before releasing "
    "no-overtake mode");

  auto local_request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
  local_costmap_clear_client_->async_send_request(
    local_request,
    [this, generation](
      rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedFuture future)
    {
      costmapClearCallback(future, true, generation);
    });

  auto global_request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
  global_costmap_clear_client_->async_send_request(
    global_request,
    [this, generation](
      rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedFuture future)
    {
      costmapClearCallback(future, false, generation);
    });
}

void PrestopNode::costmapClearCallback(
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedFuture future,
  const bool is_local,
  const uint64_t generation)
{
  if (generation != costmap_clear_generation_ || no_overtake_input_ ||
    !no_overtake_clear_pending_ || !costmap_clear_in_progress_)
  {
    return;
  }

  try {
    (void)future.get();
  } catch (const std::exception & exception) {
    costmap_clear_in_progress_ = false;
    local_costmap_clear_done_ = false;
    global_costmap_clear_done_ = false;
    local_costmap_active_confirmed_ = false;
    global_costmap_active_confirmed_ = false;
    RCLCPP_ERROR(
      get_logger(),
      "[NO_OVERTAKE_CLEAR_FAILED] Failed to clear %s costmap: %s; "
      "no-overtake mode remains active",
      is_local ? "local" : "global",
      exception.what());
    return;
  }

  if (is_local) {
    local_costmap_clear_done_ = true;
  } else {
    global_costmap_clear_done_ = true;
  }

  if (local_costmap_clear_done_ && global_costmap_clear_done_) {
    finishNoOvertakeExit();
  }
}

void PrestopNode::finishNoOvertakeExit()
{
  if (no_overtake_input_ || !no_overtake_clear_pending_) {
    return;
  }

  no_overtake_active_ = false;
  no_overtake_clear_pending_ = false;
  costmap_clear_in_progress_ = false;
  local_costmap_clear_done_ = false;
  global_costmap_clear_done_ = false;
  local_costmap_active_confirmed_ = false;
  global_costmap_active_confirmed_ = false;
  RCLCPP_INFO(
    get_logger(),
    "[NO_OVERTAKE_EXITED] Costmap clear completed; no-overtake mode released");
  publishNoOvertakeStatus();
}

bool PrestopNode::scanHasObstacleInPolygon(
  const sensor_msgs::msg::LaserScan & scan,
  const std::vector<Point2D> & polygon) const
{
  int points_in_polygon = 0;
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

    if (pointInPolygon(point, polygon)) {
      if (++points_in_polygon >= min_points_in_polygon_) {
        return true;
      }
    }
  }
  return false;
}

bool PrestopNode::pointInPolygon(
  const Point2D & point,
  const std::vector<Point2D> & polygon) const
{
  bool inside = false;
  size_t j = polygon.size() - 1;
  for (size_t i = 0; i < polygon.size(); ++i) {
    const Point2D & pi = polygon[i];
    const Point2D & pj = polygon[j];
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

void PrestopNode::publishPolygons()
{
  publishPolygon(stop_zone_polygon_pub_, stop_zone_);
  publishPolygon(no_overtake_zone_polygon_pub_, no_overtake_zone_);
}

void PrestopNode::publishPolygon(
  const rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr & publisher,
  const std::vector<Point2D> & polygon_points)
{
  if (!publisher) {
    return;
  }

  geometry_msgs::msg::PolygonStamped polygon;
  polygon.header.stamp = now();
  polygon.header.frame_id = base_frame_id_;
  polygon.polygon.points.reserve(polygon_points.size());
  for (const auto & point : polygon_points) {
    geometry_msgs::msg::Point32 polygon_point;
    polygon_point.x = static_cast<float>(point.x);
    polygon_point.y = static_cast<float>(point.y);
    polygon_point.z = 0.0f;
    polygon.polygon.points.push_back(polygon_point);
  }

  publisher->publish(polygon);
}

void PrestopNode::publishState()
{
  std_msgs::msg::String msg;
  msg.data = stateToString(state_);
  state_pub_->publish(msg);
}

void PrestopNode::publishNoOvertakeStatus()
{
  std_msgs::msg::Bool msg;
  msg.data = no_overtake_active_;
  no_overtake_status_pub_->publish(msg);
}

std::vector<Point2D> PrestopNode::parsePolygonPoints(
  const std::string & points_string,
  const std::string & parameter_name)
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
    throw std::runtime_error(parameter_name + " must contain x/y pairs");
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
    case FilterState::TIMED_STOP:
      return "TIMED_STOP";
    case FilterState::WAITING_FOR_REARM:
      return "WAITING_FOR_REARM";
    case FilterState::NO_OVERTAKE_HOLD:
      return "NO_OVERTAKE_HOLD";
  }
  return "UNKNOWN";
}

}  // namespace prestop
