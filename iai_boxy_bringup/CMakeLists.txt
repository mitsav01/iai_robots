cmake_minimum_required(VERSION 3.5)
project(iai_boxy_bringup)

find_package(ament_cmake REQUIRED)

# Install directories (if there are specific resources to install)
install(DIRECTORY config images launch rviz2
  DESTINATION share/${PROJECT_NAME}
)

# ament package declaration
ament_package()