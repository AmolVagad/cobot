cmake_minimum_required(VERSION 3.8)
project(cobot_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(proximity_sensor_node src/proximity_sensor_node.cpp)
ament_target_dependencies(proximity_sensor_node rclcpp std_msgs)

add_executable(emergency_stop_node src/emergency_stop_node.cpp)
ament_target_dependencies(emergency_stop_node rclcpp std_msgs)

install(
  TARGETS
  proximity_sensor_node
  emergency_stop_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

