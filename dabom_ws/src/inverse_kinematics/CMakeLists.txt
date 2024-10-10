cmake_minimum_required(VERSION 3.8)
project(inverse_kinematics)

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Install Python executables
install(PROGRAMS
  src/inverse_kinematics_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files into the correct directory
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Export package dependencies
ament_package()
