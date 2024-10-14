cmake_minimum_required(VERSION 3.8)
project(dabomb_description)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(urdf REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

install(DIRECTORY launch urdf meshes
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

install(DIRECTORY launch urdf meshes
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
