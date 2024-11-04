import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():


    base_xacro_file = os.path.join(get_package_share_directory('iai_boxy_base','robots'),'base.URDF.xacro')

    joint_state_publisher = Node(
		package='joint_state_publisher',
		executable='joint_state_publisher',
		name='joint_state_publisher',
		output = 'both',
	)

    robot_state_publisher = Node(
		package='robot_state_publisher',
		executable='state_publisher',
		name='robot_state_publisher',
		output='both',
		parameters=[base_xacro_file]
	)

    return LaunchDescription(
      [
        joint_state_publisher,
        robot_state_publisher
      ]
    )


<launch>
  <arg
    name="model" />
  <arg
    name="gui"
    default="False" />
  <param
    name="robot_description"
    textfile="$(find iai_boxy_base)/robots/base.URDF" />
  <param
    name="use_gui"
    value="$(arg gui)" />
  <node
    name="joint_state_publisher"
    pkg="joint_state_publisher"
    type="joint_state_publisher" />
  <node
    name="robot_state_publisher"
    pkg="robot_state_publisher"
    type="state_publisher" />
  <!--node
    name="rviz"
    pkg="rviz"
    type="rviz"
    args="-d $(find iai_boxy_base)/urdf.rviz" /-->
</launch>
