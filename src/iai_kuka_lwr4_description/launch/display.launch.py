import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    kuka_lwr4__xacro_file = os.path.join(get_package_share_directory('iai_kuka_lwr4_description'), 'urdf',
                                     'kuka_lwr_arm.urdf.xacro')
    robot_description = Command(
        [FindExecutable(name='xacro'), ' ', kuka_lwr4__xacro_file])

    rviz_file = os.path.join(get_package_share_directory('iai_kuka_lwr4_description'), 'rviz2',
                              'lwr4.rviz')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            
        ),
        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['--display-config', rviz_file])
    ])

