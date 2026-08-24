cmake_minimum_required(VERSION 3.28)
project(ugv_nav_core CXX)

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
find_package(Eigen3 REQUIRED)

# --- Module library: the C++23 named-module navigation core ------------------
# Kept internal to this package; its object code is embedded into the shared
# facade below, so downstream colcon packages never need to import modules.
add_library(ugv_nav_core_modules STATIC)
target_sources(ugv_nav_core_modules
  PRIVATE
    src/eigen_support.cpp   # link shim for Eigen free operators (see file)
  PUBLIC
    FILE_SET nav_modules TYPE CXX_MODULES FILES
      src/types.cppm
      src/math.cppm
      src/sensors.cppm
      src/eskf.cppm
      src/metrics.cppm
      src/ugv_nav_core.cppm)
target_compile_features(ugv_nav_core_modules PUBLIC cxx_std_23)
target_link_libraries(ugv_nav_core_modules PUBLIC Eigen3::Eigen)
set_target_properties(ugv_nav_core_modules PROPERTIES POSITION_INDEPENDENT_CODE ON)

# The shim must be built unoptimised so Clang emits out-of-line (vague-linkage)
# definitions of the Eigen free operators that optimised module TUs reference
# but, due to the modules linkage quirk, never define themselves.
set_source_files_properties(src/eigen_support.cpp PROPERTIES
  COMPILE_OPTIONS "-O0;-fno-inline")

# --- Shared facade: ABI-stable, include-only surface for ROS 2 nodes ---------
add_library(ugv_nav_core SHARED src/facade.cpp)
target_compile_features(ugv_nav_core PUBLIC cxx_std_23)
target_include_directories(ugv_nav_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(ugv_nav_core
  PUBLIC Eigen3::Eigen
  PRIVATE ugv_nav_core_modules)   # embedded; not part of the link interface

# --- Install / export (consumers get the header + shared library) ------------
install(DIRECTORY include/ DESTINATION include)
install(TARGETS ugv_nav_core
  EXPORT export_ugv_nav_core
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_targets(export_ugv_nav_core HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

# --- Unit tests (GoogleTest; import the module directly) ---------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest QUIET)
  set(_test_srcs
    test/test_math.cpp
    test/test_eskf.cpp
    test/test_metrics.cpp
    test/test_sensors.cpp)
  if(ament_cmake_gtest_FOUND)
    ament_add_gtest(${PROJECT_NAME}_tests ${_test_srcs})
    if(TARGET ${PROJECT_NAME}_tests)
      target_link_libraries(${PROJECT_NAME}_tests ugv_nav_core_modules)
      target_compile_features(${PROJECT_NAME}_tests PRIVATE cxx_std_23)
    endif()
  else()
    find_package(GTest REQUIRED)
    enable_testing()
    add_executable(${PROJECT_NAME}_tests ${_test_srcs})
    target_link_libraries(${PROJECT_NAME}_tests PRIVATE ugv_nav_core_modules GTest::gtest_main)
    target_compile_features(${PROJECT_NAME}_tests PRIVATE cxx_std_23)
    add_test(NAME ${PROJECT_NAME}_tests COMMAND ${PROJECT_NAME}_tests)
  endif()
endif()

ament_package()
