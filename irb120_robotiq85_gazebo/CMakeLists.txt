cmake_minimum_required(VERSION 3.5)
project(irb120_robotiq85_gazebo)

# Set C++ standard (adjust as needed)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies using ament_cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories - add additional include directories if needed
include_directories(
  ${rclcpp_INCLUDE_DIRS}
)

# Optionally, declare executables or libraries if you have any C++ nodes
# For example, if you have a node in src:
# add_executable(${PROJECT_NAME}_node src/your_node.cpp)
# ament_target_dependencies(${PROJECT_NAME}_node rclcpp std_msgs)

# Install directories for launch files, configuration, URDF, worlds, meshes, rviz, and source code
install(DIRECTORY launch config urdf worlds meshes rviz src
  DESTINATION share/${PROJECT_NAME}
)

ament_package()