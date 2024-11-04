import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():


    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "base_link", "base_footprint", "40.0"],
    )
    spawn_model_node = Node(
        package="gazebo",
        executable="spawn_model",
        name="spawn_model",
        arguments=[]
    )











# <launch>
#   <include
#     file="$(find gazebo_worlds)/launch/empty_world.launch" />
#   <node
#     name="tf_footprint_base"
#     pkg="tf"
#     type="static_transform_publisher"
#     args="0 0 0 0 0 0 base_link base_footprint 40" />
#   <node
#     name="spawn_model"
#     pkg="gazebo"
#     type="spawn_model"
#     args="-file $(find iai_boxy_base)/robots/base.URDF-urdf -model base"
#     output="screen" />
#   <include
#     file="$(find pr2_controller_manager)/controller_manager.launch" />
#   <node
#     name="fake_joint_calibration"
#     pkg="rostopic"
#     type="rostopic"
#     args="pub /calibrated std_msgs/Bool true" />
# </launch>
