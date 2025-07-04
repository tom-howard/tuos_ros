cmake_minimum_required(VERSION 3.8)
project(tuos_tb3_tools)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(cpp_executable src/minimal_node.cpp)
ament_target_dependencies(cpp_executable rclcpp)

# Install Cpp executables
install(TARGETS
  cpp_executable
  DESTINATION lib/${PROJECT_NAME}
)

## Install Python modules NOT NEEDED??
# ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/tb3_status.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
