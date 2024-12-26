cmake_minimum_required(VERSION 3.5)
project(iai_boxy_base)

# Find ament packages and dependencies
find_package(ament_cmake REQUIRED)

# Specify package dependencies here (add other dependencies as needed)
# find_package(rclcpp REQUIRED)
# find_package(sensor_msgs REQUIRED)

# Install directories (if there are specific resources to install)
install(DIRECTORY launch urdf meshes rviz2
  DESTINATION share/${PROJECT_NAME}
)

# ament package declaration
ament_package()
