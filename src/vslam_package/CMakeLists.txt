cmake_minimum_required(VERSION 3.5)
project(vslam_package)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)

# Find ROSIDL generator for message files
find_package(rosidl_default_generators REQUIRED)

# Generate message files
rosidl_generate_interfaces(vslam_package
  "msg/ArucoMarker.msg"  # Add this line for your custom message
  DEPENDENCIES std_msgs  # Include standard message dependencies
)

# Install Python executables
install(PROGRAMS
  scripts/aruco_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

