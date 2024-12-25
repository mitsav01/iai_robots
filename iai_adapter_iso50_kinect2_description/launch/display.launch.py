import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    iso50_xacro_file = os.path.join(get_package_share_directory('iai_adapter_iso50_kinect2_description'), 'urdf',
                                     'adapter_iso50_kinect2.urdf.xacro')
    robot_description = Command(
        [FindExecutable(name='xacro'), ' ', iso50_xacro_file])

    rviz_file = os.path.join(get_package_share_directory('iai_adapter_iso50_kinect2_description'), 'rviz2',
                              'default.rviz')
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
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',

        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['--display-config', rviz_file],
            output='screen'
        )
    ])
