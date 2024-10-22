cmake_minimum_required(VERSION 3.5)
project(neo_kinematics_mecanum2)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ###########
# ## Build ##
# ###########

ament_export_dependencies(std_msgs tf nav_msgs sensor_msgs)

ament_export_include_directories(common/include)

add_library(${PROJECT_NAME} common/src/Mecanum4WKinematics.cpp)

ament_export_libraries(${PROJECT_NAME})

add_executable(neo_mecanum_node ros/src/neo_kinematics_mecanum_node.cpp)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  nav_msgs
  trajectory_msgs)

target_link_libraries(neo_mecanum_node ${PROJECT_NAME})

install(DIRECTORY common/include/
   DESTINATION common/include/
)

install(TARGETS neo_mecanum_node
DESTINATION lib/${PROJECT_NAME}
	)

install(DIRECTORY launch
  	
  DESTINATION share/${PROJECT_NAME})

ament_package()