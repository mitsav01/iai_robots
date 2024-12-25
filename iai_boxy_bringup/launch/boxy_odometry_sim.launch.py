from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('iai_boxy_bringup'), 'config', 'odometrysimulationconfig.yaml'
    )

    return LaunchDescription([
        # ros2_control Node
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='ros2_control_node',
            output='screen',
            parameters=[config_file],
        ),
        
        # # Spawner for the DiffDriveController
        # Node(
        #     package='controller_manager',
        #     executable='spawner.py',
        #     arguments=['odometry_controller'],
        #     output='screen',
        # ),

        # # Optional
        # Node(
        #     package='teleop_twist_keyboard',  # For keyboard control
        #     executable='teleop_twist_keyboard',
        #     name='teleop_twist_keyboard',
        #     output='screen',
        #     remappings=[('/cmd_vel', '/odometry_controller/cmd_vel_unstamped')]
        # ),
    ])
