#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "prestop/prestop_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class NoOvertakeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  virtual std::vector<rclcpp::Parameter> extraParameterOverrides()
  {
    return {rclcpp::Parameter("noise_filter.min_points_in_polygon", 1)};
  }

  void SetUp() override
  {
    std::vector<rclcpp::Parameter> overrides{
      rclcpp::Parameter("cmd_vel_in_topic", "/test/cmd_vel_in"),
      rclcpp::Parameter("cmd_vel_out_topic", "/test/cmd_vel_out"),
      rclcpp::Parameter("scan_in_topic", "/test/scan_in"),
      rclcpp::Parameter("scan_out_topic", "/test/scan_out"),
      rclcpp::Parameter("no_overtake_topic", "/test/no_overtake"),
      rclcpp::Parameter("no_overtake_status_topic", "/test/no_overtake_status"),
      rclcpp::Parameter("local_costmap_clear_service", "/test/clear_local_costmap"),
      rclcpp::Parameter("global_costmap_clear_service", "/test/clear_global_costmap"),
      rclcpp::Parameter("local_costmap_lifecycle_service", "/test/local_costmap/get_state"),
      rclcpp::Parameter("global_costmap_lifecycle_service", "/test/global_costmap/get_state"),
      rclcpp::Parameter("state_topic", "/test/state"),
      rclcpp::Parameter("visualize_stop_zone", false),
      rclcpp::Parameter("stop_duration", 0.05),
      rclcpp::Parameter("rearm_clear_duration", 0.1),
      rclcpp::Parameter("no_overtake_default", false),
      rclcpp::Parameter("no_overtake_exit_delay", 0.0),
      rclcpp::Parameter(
        "polygons.stop_zone.points",
        "[[0.1, 0.5], [0.9, 0.5], [0.9, -0.5], [0.1, -0.5]]"),
      rclcpp::Parameter(
        "polygons.no_overtake_zone.points",
        "[[1.1, 0.5], [2.0, 0.5], [2.0, -0.5], [1.1, -0.5]]")};

    for (const auto & parameter : extraParameterOverrides()) {
      overrides.push_back(parameter);
    }

    rclcpp::NodeOptions options;
    options.parameter_overrides(overrides);

    prestop_node_ = std::make_shared<prestop::PrestopNode>(options);
    io_node_ = std::make_shared<rclcpp::Node>("prestop_test_io");
    scan_pub_ = io_node_->create_publisher<sensor_msgs::msg::LaserScan>(
      "/test/scan_in", rclcpp::SensorDataQoS());
    no_overtake_pub_ = io_node_->create_publisher<std_msgs::msg::Bool>(
      "/test/no_overtake", rclcpp::QoS(10).reliable());
    cmd_pub_ = io_node_->create_publisher<geometry_msgs::msg::Twist>(
      "/test/cmd_vel_in", 10);
    cmd_sub_ = io_node_->create_subscription<geometry_msgs::msg::Twist>(
      "/test/cmd_vel_out", 10,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        last_cmd_x_ = msg->linear.x;
        ++cmd_count_;
      });
    state_sub_ = io_node_->create_subscription<std_msgs::msg::String>(
      "/test/state", rclcpp::QoS(10).transient_local().reliable(),
      [this](std_msgs::msg::String::ConstSharedPtr msg) {last_state_ = msg->data;});
    scan_sub_ = io_node_->create_subscription<sensor_msgs::msg::LaserScan>(
      "/test/scan_out", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
        if (!msg->ranges.empty()) {
          last_scan_range_ = msg->ranges.front();
        }
        ++scan_count_;
      });
    no_overtake_status_sub_ = io_node_->create_subscription<std_msgs::msg::Bool>(
      "/test/no_overtake_status", rclcpp::QoS(10).transient_local().reliable(),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        no_overtake_status_ = msg->data;
      });

    executor_.add_node(prestop_node_);
    executor_.add_node(io_node_);
    spinFor(150ms);
  }

  void TearDown() override
  {
    executor_.remove_node(io_node_);
    executor_.remove_node(prestop_node_);
    io_node_.reset();
    prestop_node_.reset();
  }

  void spinFor(std::chrono::milliseconds duration)
  {
    const auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end) {
      executor_.spin_some();
      std::this_thread::sleep_for(2ms);
    }
  }

  void publishNoOvertake(bool active)
  {
    std_msgs::msg::Bool msg;
    msg.data = active;
    no_overtake_pub_->publish(msg);
    spinFor(30ms);
  }

  void publishObstacleScan(const float range = 0.5F)
  {
    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = io_node_->now();
    scan.header.frame_id = "base_link";
    scan.angle_min = 0.0F;
    scan.angle_max = 0.0F;
    scan.angle_increment = 1.0F;
    scan.range_min = 0.1F;
    scan.range_max = 10.0F;
    scan.ranges = {range};
    scan_pub_->publish(scan);
    spinFor(30ms);
  }

  void publishObstacleScanPoints(const int num_points, const float range = 0.5F)
  {
    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = io_node_->now();
    scan.header.frame_id = "base_link";
    scan.angle_increment = 0.02F;
    scan.angle_min = -0.5F * static_cast<float>(num_points - 1) * scan.angle_increment;
    scan.angle_max = -scan.angle_min;
    scan.range_min = 0.1F;
    scan.range_max = 10.0F;
    scan.ranges.assign(static_cast<size_t>(std::max(num_points, 0)), range);
    scan_pub_->publish(scan);
    spinFor(30ms);
  }

  void publishForwardCommand()
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.5;
    cmd_pub_->publish(cmd);
    spinFor(30ms);
  }

  void createCostmapClearServices(const bool active = true)
  {
    local_costmap_lifecycle_service_ =
      io_node_->create_service<lifecycle_msgs::srv::GetState>(
      "/test/local_costmap/get_state",
      [active](
        lifecycle_msgs::srv::GetState::Request::SharedPtr,
        lifecycle_msgs::srv::GetState::Response::SharedPtr response)
      {
        response->current_state.id = active ?
          lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE :
          lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
        response->current_state.label = active ? "active" : "inactive";
      });
    global_costmap_lifecycle_service_ =
      io_node_->create_service<lifecycle_msgs::srv::GetState>(
      "/test/global_costmap/get_state",
      [active](
        lifecycle_msgs::srv::GetState::Request::SharedPtr,
        lifecycle_msgs::srv::GetState::Response::SharedPtr response)
      {
        response->current_state.id = active ?
          lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE :
          lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
        response->current_state.label = active ? "active" : "inactive";
      });
    local_costmap_clear_service_ =
      io_node_->create_service<nav2_msgs::srv::ClearEntireCostmap>(
      "/test/clear_local_costmap",
      [this](
        nav2_msgs::srv::ClearEntireCostmap::Request::SharedPtr,
        nav2_msgs::srv::ClearEntireCostmap::Response::SharedPtr)
      {
        ++local_costmap_clear_count_;
      });
    global_costmap_clear_service_ =
      io_node_->create_service<nav2_msgs::srv::ClearEntireCostmap>(
      "/test/clear_global_costmap",
      [this](
        nav2_msgs::srv::ClearEntireCostmap::Request::SharedPtr,
        nav2_msgs::srv::ClearEntireCostmap::Response::SharedPtr)
      {
        ++global_costmap_clear_count_;
      });
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<prestop::PrestopNode> prestop_node_;
  rclcpp::Node::SharedPtr io_node_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr no_overtake_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr no_overtake_status_sub_;
  rclcpp::Service<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr local_costmap_clear_service_;
  rclcpp::Service<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr global_costmap_clear_service_;
  rclcpp::Service<lifecycle_msgs::srv::GetState>::SharedPtr local_costmap_lifecycle_service_;
  rclcpp::Service<lifecycle_msgs::srv::GetState>::SharedPtr global_costmap_lifecycle_service_;
  std::string last_state_;
  bool no_overtake_status_{false};
  double last_cmd_x_{0.0};
  float last_scan_range_{0.0F};
  size_t cmd_count_{0};
  size_t scan_count_{0};
  size_t local_costmap_clear_count_{0};
  size_t global_costmap_clear_count_{0};
};

TEST_F(NoOvertakeTest, TimedStopTransitionsToWaitingForRearm)
{
  publishNoOvertake(false);
  publishObstacleScan();
  EXPECT_EQ(last_state_, "TIMED_STOP");

  std::this_thread::sleep_for(70ms);
  publishObstacleScan();
  EXPECT_EQ(last_state_, "WAITING_FOR_REARM");

  const auto previous_count = cmd_count_;
  publishForwardCommand();
  ASSERT_GT(cmd_count_, previous_count);
  EXPECT_DOUBLE_EQ(last_cmd_x_, 0.5);
}

TEST_F(NoOvertakeTest, ObstacleFlickerDoesNotRetriggerTimedStop)
{
  publishNoOvertake(false);
  publishObstacleScan();
  std::this_thread::sleep_for(70ms);
  publishObstacleScan();
  ASSERT_EQ(last_state_, "WAITING_FOR_REARM");

  publishObstacleScan(3.0F);
  EXPECT_EQ(last_state_, "WAITING_FOR_REARM");
  publishObstacleScan();
  EXPECT_EQ(last_state_, "WAITING_FOR_REARM");
}

TEST_F(NoOvertakeTest, RearmsOnlyAfterContinuousClearDuration)
{
  publishNoOvertake(false);
  publishObstacleScan();
  std::this_thread::sleep_for(70ms);
  publishObstacleScan();
  ASSERT_EQ(last_state_, "WAITING_FOR_REARM");

  publishObstacleScan(3.0F);
  std::this_thread::sleep_for(120ms);
  publishObstacleScan(3.0F);
  EXPECT_EQ(last_state_, "CLEAR");

  publishObstacleScan();
  EXPECT_EQ(last_state_, "TIMED_STOP");
}

TEST_F(NoOvertakeTest, WaitingForRearmForwardsRawScan)
{
  publishNoOvertake(false);
  publishObstacleScan();
  std::this_thread::sleep_for(70ms);
  publishObstacleScan();
  ASSERT_EQ(last_state_, "WAITING_FOR_REARM");

  const auto previous_count = scan_count_;
  publishObstacleScan(3.0F);
  ASSERT_GT(scan_count_, previous_count);
  EXPECT_FLOAT_EQ(last_scan_range_, 3.0F);
}

TEST_F(NoOvertakeTest, StopIsHeldInsideNoOvertakeZone)
{
  publishNoOvertake(true);
  publishObstacleScan(1.5F);
  EXPECT_EQ(last_state_, "NO_OVERTAKE_HOLD");

  std::this_thread::sleep_for(1100ms);
  publishObstacleScan(1.5F);
  EXPECT_EQ(last_state_, "NO_OVERTAKE_HOLD");

  const auto previous_count = cmd_count_;
  publishForwardCommand();
  ASSERT_GT(cmd_count_, previous_count);
  EXPECT_DOUBLE_EQ(last_cmd_x_, 0.0);
}

TEST_F(NoOvertakeTest, NoOvertakePolygonIsIgnoredOutsideNoOvertakeArea)
{
  publishNoOvertake(false);
  publishObstacleScan(1.5F);

  EXPECT_EQ(last_state_, "CLEAR");
}

TEST_F(NoOvertakeTest, StopPolygonUsesTimedStopInsideNoOvertakeArea)
{
  publishNoOvertake(true);
  publishObstacleScan(0.5F);

  EXPECT_EQ(last_state_, "TIMED_STOP");
}

TEST_F(NoOvertakeTest, ClearsBothCostmapsBeforeExitingNoOvertakeZone)
{
  publishNoOvertake(true);
  publishObstacleScan(1.5F);
  ASSERT_TRUE(no_overtake_status_);

  publishNoOvertake(false);
  publishObstacleScan(1.5F);
  EXPECT_TRUE(no_overtake_status_);
  EXPECT_EQ(local_costmap_clear_count_, 0U);
  EXPECT_EQ(global_costmap_clear_count_, 0U);

  createCostmapClearServices();
  spinFor(30ms);
  publishObstacleScan(1.5F);
  spinFor(30ms);

  EXPECT_EQ(local_costmap_clear_count_, 1U);
  EXPECT_EQ(global_costmap_clear_count_, 1U);
  EXPECT_FALSE(no_overtake_status_);
}

TEST_F(NoOvertakeTest, KeepsNoOvertakeActiveWhileCostmapsAreInactive)
{
  createCostmapClearServices(false);
  spinFor(30ms);

  publishNoOvertake(true);
  publishObstacleScan(1.5F);
  publishNoOvertake(false);
  publishObstacleScan(1.5F);
  spinFor(30ms);

  EXPECT_EQ(local_costmap_clear_count_, 0U);
  EXPECT_EQ(global_costmap_clear_count_, 0U);
  EXPECT_TRUE(no_overtake_status_);
}

class NoiseFilterTest : public NoOvertakeTest
{
protected:
  std::vector<rclcpp::Parameter> extraParameterOverrides() override
  {
    return {rclcpp::Parameter("noise_filter.min_points_in_polygon", 2)};
  }
};

TEST_F(NoiseFilterTest, SingleNoisePointDoesNotTriggerTimedStop)
{
  publishNoOvertake(false);
  publishObstacleScanPoints(1, 0.5F);

  EXPECT_EQ(last_state_, "CLEAR");
}

TEST_F(NoiseFilterTest, EnoughPointsInPolygonTriggersTimedStop)
{
  publishNoOvertake(false);
  publishObstacleScanPoints(2, 0.5F);

  EXPECT_EQ(last_state_, "TIMED_STOP");
}
