cmake_minimum_required(VERSION 3.8)
project(tuos_task_sims)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ros_gz_sim REQUIRED)
# find_package(turtlebot3_gazebo REQUIRED)
find_package(tuos_simulations REQUIRED)

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
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(gazebo_ros_pkgs)
ament_export_dependencies(turtlebot3_gazebo)
ament_export_dependencies(tuos_simulations)

ament_package()
