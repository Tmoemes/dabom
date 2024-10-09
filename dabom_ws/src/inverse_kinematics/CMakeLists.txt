cmake_minimum_required(VERSION 3.8)
project(inverse_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Install Python executables
install(PROGRAMS
  src/inverse_kinematics_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install additional resources (e.g., launch files, if needed)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# This is required to export the package as ament_cmake
ament_package()