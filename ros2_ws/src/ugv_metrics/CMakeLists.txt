cmake_minimum_required(VERSION 3.28)
project(ugv_metrics CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ugv_nav_core REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(metrics_node src/metrics_node.cpp)
target_compile_features(metrics_node PRIVATE cxx_std_23)
ament_target_dependencies(metrics_node rclcpp nav_msgs geometry_msgs ugv_nav_core)
target_link_libraries(metrics_node nlohmann_json::nlohmann_json)

install(TARGETS metrics_node DESTINATION lib/${PROJECT_NAME})

ament_package()
