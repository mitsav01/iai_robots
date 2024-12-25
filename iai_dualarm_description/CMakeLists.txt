cmake_minimum_required(VERSION 3.5)
project(iai_dualarm_description)

find_package(ament_cmake REQUIRED)


install(DIRECTORY launch robots config rviz2
  DESTINATION share/${PROJECT_NAME}
)

# ament package declaration
ament_package()
