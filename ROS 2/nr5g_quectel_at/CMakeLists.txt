cmake_minimum_required(VERSION 3.5)
project(nr5g_quectel_at)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Add custom message files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ATQuectelNR5G.msg"
  DEPENDENCIES std_msgs
)

# Install Python node
install(PROGRAMS
  src/nr5gquectel.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
