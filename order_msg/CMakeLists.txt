# Specify the minimum CMake version required
cmake_minimum_required(VERSION 3.5)

# Define the project name
project(order_msg)

# Set the C standard to C99 if not already set
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Set the C++ standard to C++14 if not already set
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# If using GNU C++ compiler or Clang, add certain compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find and load settings of required packages
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom ROS interfaces (messages, services, actions)
# In this case, it generates interfaces for "Order.msg"
# and states that they have dependencies on geometry_msgs
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Order.msg"
  DEPENDENCIES geometry_msgs
)

# Export dependencies for other packages to use when they depend on this package
ament_export_dependencies(rosidl_default_runtime)

# Package the project
ament_package()
