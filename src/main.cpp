#include <memory>

#include "prestop/prestop_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<prestop::PrestopNode>());
  rclcpp::shutdown();
  return 0;
}
