cmake_minimum_required(VERSION 3.5)
project(iai_kuka_lwr4_description)

# Find ament packages and dependencies
find_package(ament_cmake REQUIRED)



# Install directories (if there are specific resources to install)
install(DIRECTORY launch robots meshes rviz2 urdf defs
  DESTINATION share/${PROJECT_NAME}
)

# ament package declaration
ament_package()


