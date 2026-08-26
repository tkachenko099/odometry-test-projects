cmake_minimum_required(VERSION 3.28)
project(ugv_return_home CXX)

# --- Toolchain policy: C++23 with named modules (Clang + Ninja) --------------
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_SCAN_FOR_MODULES ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
# Optional: real MAVLink backend (ArduPilot SITL / hardware-in-the-loop).
find_package(mavros_msgs QUIET)

# --- Module library: the C++23 named-module failsafe core --------------------
# Kept internal to this package; embedded into the shared facade so downstream
# packages never need to import modules (Eigen-free, so no link shim needed).
add_library(ugv_return_home_modules STATIC)
target_sources(ugv_return_home_modules
  PUBLIC
    FILE_SET rth_modules TYPE CXX_MODULES FILES
      src/types.cppm
      src/geo.cppm
      src/route.cppm
      src/simplify.cppm
      src/link.cppm
      src/mission.cppm
      src/ugv_rth.cppm)
target_compile_features(ugv_return_home_modules PUBLIC cxx_std_23)
set_target_properties(ugv_return_home_modules PROPERTIES POSITION_INDEPENDENT_CODE ON)

# --- Shared facade: ABI-stable, include-only surface for ROS 2 nodes ---------
add_library(ugv_return_home SHARED src/facade.cpp)
target_compile_features(ugv_return_home PUBLIC cxx_std_23)
target_include_directories(ugv_return_home PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(ugv_return_home PRIVATE ugv_return_home_modules)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ugv_return_home
  EXPORT export_ugv_return_home
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)
ament_export_targets(export_ugv_return_home HAS_LIBRARY_TARGET)

# --- Standalone demo (no ROS): drives the full failsafe scenario -------------
add_executable(rth_demo src/rth_demo.cpp)
target_compile_features(rth_demo PRIVATE cxx_std_23)
target_link_libraries(rth_demo PRIVATE ugv_return_home)

# --- ROS 2 node: bridges the controller to MAVLink/mavros topics ------------
add_executable(rth_node src/rth_node.cpp)
target_compile_features(rth_node PRIVATE cxx_std_23)
target_link_libraries(rth_node ugv_return_home)  # plain form (ament uses plain)
ament_target_dependencies(rth_node
  rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs std_srvs)
if(mavros_msgs_FOUND)
  message(STATUS "ugv_return_home: mavros_msgs found -> MavrosLink backend enabled")
  target_compile_definitions(rth_node PRIVATE HAVE_MAVROS)
  ament_target_dependencies(rth_node mavros_msgs)
endif()

install(TARGETS rth_demo rth_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# --- Unit tests (GoogleTest; import the module directly) ---------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest QUIET)
  set(_test_srcs
    test/test_geo.cpp
    test/test_route.cpp
    test/test_simplify.cpp
    test/test_link.cpp
    test/test_mission.cpp)
  if(ament_cmake_gtest_FOUND)
    ament_add_gtest(${PROJECT_NAME}_tests ${_test_srcs})
    if(TARGET ${PROJECT_NAME}_tests)
      target_link_libraries(${PROJECT_NAME}_tests ugv_return_home_modules)
      target_compile_features(${PROJECT_NAME}_tests PRIVATE cxx_std_23)
    endif()
  else()
    find_package(GTest REQUIRED)
    enable_testing()
    add_executable(${PROJECT_NAME}_tests ${_test_srcs})
    target_link_libraries(${PROJECT_NAME}_tests PRIVATE ugv_return_home_modules GTest::gtest_main)
    target_compile_features(${PROJECT_NAME}_tests PRIVATE cxx_std_23)
    add_test(NAME ${PROJECT_NAME}_tests COMMAND ${PROJECT_NAME}_tests)
  endif()
endif()

ament_package()
