cmake_minimum_required(VERSION 3.16)
project(vsac4gaming LANGUAGES C CXX)

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
find_package(geometry_msgs QUIET)
find_package(rosidl_default_generators QUIET)
find_package(rosidl_typesupport_fastrtps_cpp QUIET)

if(ament_cmake_FOUND AND rclcpp_FOUND AND std_msgs_FOUND AND geometry_msgs_FOUND AND rosidl_default_generators_FOUND AND rosidl_typesupport_fastrtps_cpp_FOUND)
    message(STATUS "Building ROS2 node: vsac_app")

    rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/VehicleState.msg"
        DEPENDENCIES geometry_msgs
    )

    add_executable(vsac_app src/main.cpp)
    target_link_libraries(vsac_app vsac)
    ament_target_dependencies(vsac_app rclcpp std_msgs geometry_msgs)

    rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
    target_link_libraries(vsac_app ${cpp_typesupport_target})

    # Allow the executable to locate generated typesupport libraries in the build dir.
    set_target_properties(vsac_app PROPERTIES
        BUILD_RPATH "$ORIGIN"
        INSTALL_RPATH "$ORIGIN/.."
    )

    install(TARGETS vsac_app
        DESTINATION lib/${PROJECT_NAME}
    )

    ament_export_dependencies(rosidl_default_runtime)
    ament_package()
else()
    message(STATUS "ROS2 not found - skipping vsac_app")
endif()
