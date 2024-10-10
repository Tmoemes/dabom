cmake_minimum_required(VERSION 3.5)
project(odom)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Install Python executables
install(PROGRAMS
  src/odom_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
