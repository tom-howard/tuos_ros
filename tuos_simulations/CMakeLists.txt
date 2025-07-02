cmake_minimum_required(VERSION 3.8)
project(tuos_simulations)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
# find_package(gazebo REQUIRED)
find_package(ros_gz_sim REQUIRED)
# find_package(turtlebot3_gazebo REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(nav_msgs REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(rclpy REQUIRED)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
)

install(DIRECTORY 
  launch
  models
  worlds
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  ros_gz_sim
)

ament_package()
