cmake_minimum_required(VERSION 3.28)
project(ugv_gazebo NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds config launch
        DESTINATION share/${PROJECT_NAME})

ament_package()
