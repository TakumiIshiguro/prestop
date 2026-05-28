#ifndef PRESTOP__PRESTOP_NODE_HPP_
#define PRESTOP__PRESTOP_NODE_HPP_

#include <string>
#include <vector>

#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace prestop
{

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

enum class FilterState
{
  CLEAR,
  STOP
};

class PrestopNode : public rclcpp::Node
{
public:
  PrestopNode();

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  bool scanHasObstacleInStopZone(const sensor_msgs::msg::LaserScan & scan) const;
  bool pointInPolygon(const Point2D & point) const;
  sensor_msgs::msg::LaserScan makeEmptyScan(const sensor_msgs::msg::LaserScan & scan) const;
  void publishZeroCmdVel();
  void publishStopZonePolygon();

  static std::vector<Point2D> parsePolygonPoints(const std::string & points_string);
  static std::string stateToString(FilterState state);

  std::string cmd_vel_in_topic_{"/cmd_vel_raw"};
  std::string cmd_vel_out_topic_{"/cmd_vel"};
  std::string scan_in_topic_{"/scan_raw"};
  std::string scan_out_topic_{"/scan"};
  std::string empty_scan_mode_{"inf"};
  std::string base_frame_id_{"base_link"};
  std::string stop_zone_polygon_topic_{"/prestop/stop_zone_polygon"};

  double stop_duration_{3.0};
  double min_obstacle_scan_duration_{0.0};
  bool stop_zone_enabled_{true};
  bool visualize_stop_zone_{true};
  std::string stop_zone_action_type_{"stop"};
  std::vector<Point2D> stop_zone_;

  FilterState state_{FilterState::CLEAR};
  bool has_scan_{false};
  bool waiting_for_clear_{false};
  bool has_obstacle_scan_{false};
  rclcpp::Time stop_start_time_;
  rclcpp::Time last_obstacle_scan_time_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr stop_zone_polygon_pub_;
  rclcpp::TimerBase::SharedPtr polygon_timer_;
};

}  // namespace prestop

#endif  // PRESTOP__PRESTOP_NODE_HPP_
