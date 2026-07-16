cmake_minimum_required(VERSION 3.8)
project(prestop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(prestop_core src/prestop_node.cpp)
target_include_directories(prestop_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(prestop_core PUBLIC cxx_std_17)
ament_target_dependencies(
  prestop_core
  geometry_msgs
  lifecycle_msgs
  nav2_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

add_executable(prestop_node src/main.cpp)
target_link_libraries(prestop_node prestop_core)
ament_target_dependencies(prestop_node rclcpp)

install(TARGETS
  prestop_core
  prestop_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_no_overtake test/test_no_overtake.cpp)
  target_link_libraries(test_no_overtake prestop_core)
  ament_target_dependencies(
    test_no_overtake
    geometry_msgs
    lifecycle_msgs
    nav2_msgs
    rclcpp
    sensor_msgs
    std_msgs
  )
endif()

ament_package()
