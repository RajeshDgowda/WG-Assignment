# Specify the minimum CMake version required
cmake_minimum_required(VERSION 3.5)
# Define the project name and version
project(amr_systems)

# Set the C standard to C99 if not already set
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Set the C++ standard to C++17 if not already set
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# If using GNU C++ compiler or Clang, add certain compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find and load settings of required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(order_msg REQUIRED)

# Specify the include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${order_msg_INCLUDE_DIRS}
)

# Add the executable for the order_optimizer_node
add_executable(order_optimizer_node src/order_optimizer.cpp)
# Link the yaml-cpp library to the executable
target_link_libraries(order_optimizer_node yaml-cpp)

# Specify dependencies for the target
ament_target_dependencies(order_optimizer_node
  rclcpp
  geometry_msgs
  visualization_msgs
  order_msg
  yaml-cpp
)

# Export the dependencies for other packages to use
ament_export_dependencies(
  geometry_msgs
  visualization_msgs
  order_msg
  yaml-cpp
  rclcpp)

# If testing is enabled, add testing configurations
if(BUILD_TESTING)
  set(SOURCES src/order_optimizer.cpp)
  # Create a shared library for testing purposes
  add_library(amr_systems SHARED ${SOURCES})

  # Find and load settings of required packages for testing
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(yaml-cpp REQUIRED)
  find_package(visualization_msgs REQUIRED)
  find_package(order_msg REQUIRED)

  # Automatically find and load test dependencies
  ament_lint_auto_find_test_dependencies()

  # Add a test executable
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(order_optimizer_test test/test_order_optimizer.cpp)
  # Specify dependencies for the test
  ament_target_dependencies(amr_systems rclcpp visualization_msgs order_msg)
  target_link_libraries(order_optimizer_test ${PROJECT_NAME} yaml-cpp)
endif()

# Package the project
ament_package()

# Installation rules for headers, libraries, and binaries
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
)

install(
  TARGETS order_optimizer_node
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
