import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # Define paths to necessary files
    kevin_xacro_file = os.path.join(get_package_share_directory('iai_urg_kevin_robot_description'), 'urdf', 'kevin.urdf')
    rviz_file = os.path.join(get_package_share_directory('iai_urg_kevin_robot_description'), 'rviz2', 'default.rviz')

    # Command to process the URDF file with xacro
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', kevin_xacro_file]),
        value_type=str
    )

    # Return the launch description with nodes for robot state publisher, joint state publisher GUI, and RViz
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['--display-config', rviz_file],
            output='screen'
        )
    ])
