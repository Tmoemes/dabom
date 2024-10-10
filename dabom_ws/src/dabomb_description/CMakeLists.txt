cmake_minimum_required(VERSION 3.8)
project(dabomb_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(gazebo_ros)  # Required for Gazebo simulation
find_package(robot_state_publisher REQUIRED)  # Publishes URDF to TF
find_package(urdf REQUIRED)  # For working with URDF files
find_package(rclcpp REQUIRED)  # Required if using rclcpp nodes
# Optional: Add ros2_control if controlling hardware:
# find_package(ros2_control REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install directories
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY urdf/ DESTINATION share/${PROJECT_NAME}/urdf)  # Install urdf directory
install(FILES 
    meshes/base_link.stl
    meshes/LIDAR_1.stl
    meshes/Wheel1_1.stl
    meshes/Wheel2_1.stl
    meshes/Wheel3_1.stl
    meshes/Wheel4_1.stl
    DESTINATION share/${PROJECT_NAME}/meshes)

install(DIRECTORY resource/ DESTINATION share/${PROJECT_NAME}/resource)  # Install resource directory

ament_package()
