cmake_minimum_required(VERSION 3.5)
project(robot_navigation)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(teb_local_planner REQUIRED)

add_executable(icp_localization src/icp_localization.cpp)
ament_target_dependencies(icp_localization rclcpp sensor_msgs pcl_ros teb_local_planner)

install(TARGETS
  icp_localization
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()