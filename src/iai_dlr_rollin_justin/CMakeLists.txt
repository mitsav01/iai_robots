cmake_minimum_required(VERSION 3.5)
project(iai_dlr_rollin_justin)

find_package(ament_cmake REQUIRED)

install(DIRECTORY fbx mjcf urdf
  DESTINATION share/${PROJECT_NAME}
)

# ament package declaration
ament_package()