cmake_minimum_required(VERSION 3.16)
project(VehicleStateAndControlForGaming LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(vsac
    src/virtual_pad.cpp
    src/mapping.cpp
)

target_include_directories(vsac PUBLIC
    src
    external/pine/src
)

target_compile_options(vsac PRIVATE -Wall -Wextra -Wpedantic)

find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(std_msgs QUIET)

if(ament_cmake_FOUND AND rclcpp_FOUND AND std_msgs_FOUND)
    message(STATUS "Building ROS2 node: vsac_app")
    add_executable(vsac_app src/main.cpp)
    target_link_libraries(vsac_app vsac)
    ament_target_dependencies(vsac_app rclcpp std_msgs)
else()
    message(STATUS "ROS2 not found - skipping vsac_app")
endif()
