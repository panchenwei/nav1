#!/bin/bash


source /opt/ros/humble/setup.bash


source ~/robot_navigation/install/setup.bash


ros2 launch livox_ros_driver2 livox_lidar_launch.py &


ros2 launch fastlio fastlio_launch.py &


ros2 launch robot_navigation bringup_launch.py &


wait