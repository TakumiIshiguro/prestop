#ifndef PRESTOP__PRESTOP_NODE_HPP_
#define PRESTOP__PRESTOP_NODE_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

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
  TIMED_STOP,
  NO_OVERTAKE_HOLD
};

class PrestopNode : public rclcpp::Node
{
public:
  explicit PrestopNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void noOvertakeCallback(const std_msgs::msg::Bool::SharedPtr msg);

  bool scanHasObstacleInPolygon(
    const sensor_msgs::msg::LaserScan & scan,
    const std::vector<Point2D> & polygon) const;
  bool pointInPolygon(
    const Point2D & point,
    const std::vector<Point2D> & polygon) const;
  void updateNoOvertakeState(const rclcpp::Time & current_time);
  void requestCostmapClear();
  void costmapLifecycleCallback(
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture future,
    bool is_local,
    uint64_t generation);
  void sendCostmapClearRequests(uint64_t generation);
  void costmapClearCallback(
    rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedFuture future,
    bool is_local,
    uint64_t generation);
  void finishNoOvertakeExit();
  sensor_msgs::msg::LaserScan makeEmptyScan(const sensor_msgs::msg::LaserScan & scan) const;
  void publishZeroCmdVel();
  void publishPolygons();
  void publishPolygon(
    const rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr & publisher,
    const std::vector<Point2D> & polygon);
  void publishState();
  void publishNoOvertakeStatus();

  static std::vector<Point2D> parsePolygonPoints(
    const std::string & points_string,
    const std::string & parameter_name);
  static std::string stateToString(FilterState state);

  std::string cmd_vel_in_topic_{"/cmd_vel_raw"};
  std::string cmd_vel_out_topic_{"/cmd_vel"};
  std::string scan_in_topic_{"/scan_raw"};
  std::string scan_out_topic_{"/scan"};
  std::string empty_scan_mode_{"inf"};
  std::string base_frame_id_{"base_link"};
  std::string stop_zone_polygon_topic_{"/prestop/stop_zone_polygon"};
  std::string no_overtake_zone_polygon_topic_{"/prestop/no_overtake_zone_polygon"};
  std::string no_overtake_topic_{"/no_overtake_active"};
  std::string no_overtake_status_topic_{"/prestop/no_overtake_active"};
  std::string state_topic_{"/prestop/state"};
  std::string local_costmap_clear_service_{
    "/local_costmap/clear_entirely_local_costmap"};
  std::string global_costmap_clear_service_{
    "/global_costmap/clear_entirely_global_costmap"};
  std::string local_costmap_lifecycle_service_{"/controller_server/get_state"};
  std::string global_costmap_lifecycle_service_{"/planner_server/get_state"};

  double stop_duration_{3.0};
  double min_obstacle_scan_duration_{0.0};
  double no_overtake_exit_delay_{1.0};
  bool stop_zone_enabled_{true};
  bool no_overtake_zone_enabled_{true};
  bool visualize_stop_zone_{true};
  bool no_overtake_input_{true};
  bool no_overtake_active_{true};
  bool no_overtake_clear_pending_{false};
  bool clear_costmaps_before_no_overtake_exit_{true};
  bool costmap_clear_in_progress_{false};
  bool local_costmap_clear_done_{false};
  bool global_costmap_clear_done_{false};
  bool local_costmap_active_confirmed_{false};
  bool global_costmap_active_confirmed_{false};
  uint64_t costmap_clear_generation_{0};
  std::string stop_zone_action_type_{"stop"};
  std::string no_overtake_zone_action_type_{"no_overtake"};
  std::vector<Point2D> stop_zone_;
  std::vector<Point2D> no_overtake_zone_;

  FilterState state_{FilterState::CLEAR};
  bool has_scan_{false};
  bool waiting_for_clear_{false};
  bool has_obstacle_scan_{false};
  rclcpp::Time stop_start_time_;
  rclcpp::Time last_obstacle_scan_time_;
  rclcpp::Time no_overtake_clear_start_time_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr no_overtake_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr stop_zone_polygon_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr
    no_overtake_zone_polygon_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr no_overtake_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr local_costmap_clear_client_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr global_costmap_clear_client_;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr local_costmap_lifecycle_client_;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr global_costmap_lifecycle_client_;
  rclcpp::TimerBase::SharedPtr polygon_timer_;
};

}  // namespace prestop

#endif  // PRESTOP__PRESTOP_NODE_HPP_
