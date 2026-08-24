cmake_minimum_required(VERSION 3.28)
project(ugv_bringup NONE)

find_package(ament_cmake REQUIRED)

install(PROGRAMS ugv_bringup/trajectory_commander.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME trajectory_commander)

install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME})

ament_package()
