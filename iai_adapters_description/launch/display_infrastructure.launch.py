import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

	lwr4_xacro_file = os.path.join(get_package_share_directory('iai_adapters_description'),'urdf','lwr4_cabling_adaptor.urdf.xacro')


	joint_state_publisher = Node(
		package='joint_state_publisher',
		executable='joint_state_publisher',
		name='joint_state_publisher',
		output = 'both',
	)

	robot_state_publisher = Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		name='robot_state_publisher',
		output='both',
		parameters=[lwr4_xacro_file]
	)

	rviz_base = os.path.join(get_package_share_directory('iai_adapters_description'))
	rviz_full_config = os.path.join(rviz_base, 'urdf.rviz')

	rviz_node = Node(
		package="rviz2",
		executable="rviz2",
		name="rviz2",
		output="log",
		arguments=["-d", rviz_full_config],
		parameters=[lwr4_xacro_file]
	)


	return LaunchDescription(
		[joint_state_publisher,
		robot_state_publisher,
        rviz_node
		]
	)
        