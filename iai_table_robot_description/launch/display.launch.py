import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Paths
    iai_table_robot_description_path = os.path.join(get_package_share_directory('iai_table_robot_description'),'urdf','ur5_table.urdf.xacro')
    robot_description = Command(
        [FindExecutable(name='xacro'), ' ', iai_table_robot_description_path])
    display_config_path = os.path.join(iai_table_robot_description_path, 'config', 'display.rviz')

    return LaunchDescription([
        
        # Joint State Publisher with GUI and joint offsets
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher',
            parameters=[{
                'use_gui': True,
                'zeros': {
                    'shoulder_pan_joint': 0.7854,
                    'shoulder_lift_joint': -0.78,
                    'elbow_joint': 0.78,
                    'wrist_1_joint': -1.57,
                    'wrist_2_joint': -1.57
                }
            }]
        ),
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', display_config_path],
            output='screen'
        )
    ])
