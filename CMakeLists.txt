cmake_minimum_required(VERSION 3.8)
project(quad_off_board)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Add compile options, if using GCC or Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED) 
find_package(nav_msgs REQUIRED) 
find_package(visualization_msgs REQUIRED) 
# Add your executable
# add_executable(off_board src/control.cpp)
# add_executable(vel_control src/vel_control.cpp)
add_executable(viz src/visualizer.cpp)
ament_target_dependencies(viz rclcpp sensor_msgs geometry_msgs px4_msgs nav_msgs visualization_msgs)
# Specify dependencies for your executable
# ament_target_dependencies(viz
#   rclcpp
#   px4_msgs
# )

# Install the executable
install(TARGETS
  viz
  DESTINATION lib/${PROJECT_NAME}
)

# Necessary for ROS2 package
ament_package()