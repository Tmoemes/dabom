cmake_minimum_required(VERSION 3.8)
project(unique_joint_state_publisher)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)  # For Python-based nodes
find_package(rclpy REQUIRED)  # Python, not rclcpp
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Install Python scripts
install(PROGRAMS
  src/unique_joint_state_publisher_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Export dependencies
ament_export_dependencies(rclpy sensor_msgs geometry_msgs)

# Finalize the package
ament_package()
