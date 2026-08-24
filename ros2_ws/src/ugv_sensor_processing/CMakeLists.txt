cmake_minimum_required(VERSION 3.28)
project(ugv_sensor_processing CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ugv_nav_core REQUIRED)

add_executable(wheel_odometry_node src/wheel_odometry_node.cpp)
target_compile_features(wheel_odometry_node PRIVATE cxx_std_23)
ament_target_dependencies(wheel_odometry_node rclcpp sensor_msgs geometry_msgs ugv_nav_core)

install(TARGETS wheel_odometry_node DESTINATION lib/${PROJECT_NAME})

ament_package()
