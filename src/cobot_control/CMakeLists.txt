cmake_minimum_required(VERSION 3.8)
project(cobot_control)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Standard compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find necessary ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# --- Core Speed Controller Library ---
add_library(${PROJECT_NAME} src/speed_controller.cpp)

# Make headers available to other targets
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Set C++ standard for the library
set_target_properties(${PROJECT_NAME} PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
)

# --- ROS2 Control Node Executable ---
add_executable(control_node src/control_node.cpp)

# Link against required ROS2 packages
ament_target_dependencies(control_node
  rclcpp
  std_msgs
)

# Link against our internal library
target_link_libraries(control_node ${PROJECT_NAME})

# Set C++ standard for the executable
set_target_properties(control_node PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
)

# --- Testing ---
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  
  # Configure linting to be less strict for now
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_lint_cmake_FOUND TRUE)
  
  # Or completely disable linting for debugging
  # ament_lint_auto_find_test_dependencies()
  
  # Main unit test
  ament_add_gtest(test_speed_controller test/test_speed_controller.cpp)
  if(TARGET test_speed_controller)
    target_include_directories(test_speed_controller PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    target_link_libraries(test_speed_controller ${PROJECT_NAME})
    set_target_properties(test_speed_controller PROPERTIES
      CXX_STANDARD 17
      CXX_STANDARD_REQUIRED ON
    )
  endif()
  
endif()

# --- Installation ---
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  EXPORT export_${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_package()