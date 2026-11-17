cmake_minimum_required(VERSION 3.28)
project(ugv_localization CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Quality gate: warnings on everywhere, errors on request (CI sets it) ----
option(UGV_WERROR "Treat compiler warnings as errors" OFF)
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion)
  if(UGV_WERROR)
    add_compile_options(-Werror)
  endif()
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ugv_nav_core REQUIRED)

add_executable(eskf_node src/eskf_node.cpp)
target_compile_features(eskf_node PRIVATE cxx_std_23)
ament_target_dependencies(eskf_node
  rclcpp sensor_msgs nav_msgs geometry_msgs ugv_nav_core)

install(TARGETS eskf_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
