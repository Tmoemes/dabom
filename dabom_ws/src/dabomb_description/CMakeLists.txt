cmake_minimum_required(VERSION 3.8)
project(dabomb_description)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(urdf REQUIRED)

# Install URDF, launch, and mesh files
install(DIRECTORY launch urdf meshes
  DESTINATION share/${PROJECT_NAME}/
)

# Install the resource directory (if it contains files like package manifest)
install(DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}/
)

# Mark the package for export
ament_package()
