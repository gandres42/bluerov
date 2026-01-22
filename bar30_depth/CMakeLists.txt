cmake_minimum_required(VERSION 3.8)
project(bar30_depth)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Depth.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)

# Install Python scripts
install(PROGRAMS
  scripts/bar30_depth_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
