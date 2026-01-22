cmake_minimum_required(VERSION 3.8)
project(rti_dvl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DVL.msg"
  "msg/Command.msg"
  "msg/BottomTrack.msg"
  DEPENDENCIES geometry_msgs std_msgs builtin_interfaces
)

# Install Python scripts
install(PROGRAMS
  scripts/rti_dvl_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install pynmea2 module
install(DIRECTORY
  scripts/pynmea2
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
