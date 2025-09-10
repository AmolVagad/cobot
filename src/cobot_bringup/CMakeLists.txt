cmake_minimum_required(VERSION 3.8)
project(cobot_bringup)

# Standard compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find necessary ROS2 packages
find_package(ament_cmake REQUIRED)

# Install the launch directory to the package's shared space
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()