cmake_minimum_required(VERSION 3.28)
project(ugv_description NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch rviz
        DESTINATION share/${PROJECT_NAME})

ament_package()
