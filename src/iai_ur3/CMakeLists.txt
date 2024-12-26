cmake_minimum_required(VERSION 3.5)
project(iai_ur3)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch robots rviz2 meshes
  DESTINATION share/${PROJECT_NAME}
)

# ament package declaration
ament_package()
