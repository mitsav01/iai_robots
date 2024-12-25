from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define paths to necessary files
    moveit_config_path = get_package_share_directory('your_moveit_package')
    moveit_config_launch = os.path.join(moveit_config_path, 'launch', 'move_group.launch.py')

    return LaunchDescription([
        # Launch the MoveIt 2 move_group node with kinematics configuration
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(moveit_config_launch),
            launch_arguments={
                'config_file': os.path.join(moveit_config_path, 'config', 'kinematics.yaml')
            }.items()
        ),

        # Optionally, launch RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(moveit_config_path, 'config', 'moveit.rviz')]
        ),
    ])
