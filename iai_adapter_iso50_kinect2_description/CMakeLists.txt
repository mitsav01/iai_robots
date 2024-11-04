cmake_minimum_required(VERSION 3.5)
project(iai_adapter_iso50_kinect2_description)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Uncomment and find additional dependencies as needed
# find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################

# Uncomment and modify as needed to declare messages, services, or actions
# find_package(rosidl_default_generators REQUIRED)

# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/Message1.msg"
#   "srv/Service1.srv"
#   "action/Action1.action"
#   DEPENDENCIES std_msgs
# )

###################################
## ament specific configuration ##
###################################
ament_package(
  # Uncomment if your package has include directories
  # INCLUDE_DIRS include
  # LIBRARIES iai_kms_40_description
  # CATKIN_DEPENDS other_catkin_pkg
  # DEPENDS system_lib
)

###########
## Build ##
###########

# Specify additional locations of header files
# Your package locations should be listed before other locations
# include_directories(include)

# Declare a cpp library
# add_library(iai_kms_40_description
#   src/${PROJECT_NAME}/iai_kms_40_description.cpp
# )

# Declare a cpp executable
# add_executable(iai_kms_40_description_node src/iai_kms_40_description_node.cpp)

# Add target dependencies of the executable/library
# as an example, message headers may need to be generated before nodes
# add_dependencies(iai_kms_40_description_node iai_kms_40_description_generate_messages_cpp)

# Specify libraries to link a library or executable target against
# target_link_libraries(iai_kms_40_description_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use ament DESTINATION variables
# See https://docs.ros.org/en/foxy/How-To-Guides/Creating-a-CMake-Package.html#installing

# Mark executable scripts (Python etc.) for installation
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${AMENT_PACKAGE_BIN_DESTINATION}
# )

# Mark executables and/or libraries for installation
# install(TARGETS iai_kms_40_description iai_kms_40_description_node
#   EXPORT export_${PROJECT_NAME}
#   ARCHIVE DESTINATION lib/${PROJECT_NAME}
#   LIBRARY DESTINATION lib/${PROJECT_NAME}
#   RUNTIME DESTINATION bin
# )

# Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION include/${PROJECT_NAME}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

# Mark other files for installation (e.g., launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION share/${PROJECT_NAME}
# )

install(
  DIRECTORY meshes urdf
  DESTINATION share/${PROJECT_NAME}
)
#############
## Testing ##
#############

# Add gtest based cpp test target and link libraries
# find_package(ament_lint_auto REQUIRED)
# find_package(ament_lint_common REQUIRED)

# ament_add_gtest(${PROJECT_NAME}-test test/test_iai_kms_40_description.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

# ament_add_nosetests(test)

# Finalize the package
ament_export_dependencies(rclcpp)
ament_package()