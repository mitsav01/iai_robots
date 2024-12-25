from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define paths to included launch files
    boxy_loopback_sim_no_controllers_launch = os.path.join(
        get_package_share_directory('iai_boxy_bringup'), 'launch', 'boxy_loopback_sim_no_controllers.launch.py'
    )
    boxy_odometry_sim_launch = os.path.join(
        get_package_share_directory('iai_boxy_bringup'), 'launch', 'boxy_odometry_sim.launch.py'
    )
    boxy_sim_vel_controllers_launch = os.path.join(
        get_package_share_directory('iai_boxy_controller_configuration'), 'launch', 'boxy_sim_vel_controllers.launch.py'
    )
    boxy_pos_controllers_launch = os.path.join(
        get_package_share_directory('iai_boxy_controller_configuration'), 'launch', 'boxy_pos_controllers.launch.py'
    )
    boxy_gripper_controller_launch = os.path.join(
        get_package_share_directory('iai_boxy_controller_configuration'), 'launch', 'boxy_gripper_controller.launch.py'
    )

    return LaunchDescription([
        # Declare the argument for conditional parameter loading
        DeclareLaunchArgument(
            'upload_default_start_config',
            default_value='true',
            description='Upload default start configuration'
        ),

        # Include boxy_loopback_sim_no_controllers.launch with argument
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(boxy_loopback_sim_no_controllers_launch),
            launch_arguments={'upload-default-start-config': LaunchConfiguration('upload_default_start_config')}.items()
        ),

        # Include other necessary launch files
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(boxy_odometry_sim_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(boxy_sim_vel_controllers_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(boxy_pos_controllers_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(boxy_gripper_controller_launch)
        ),

        # Robot State Publisher Node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='log'
        ),
    ])
