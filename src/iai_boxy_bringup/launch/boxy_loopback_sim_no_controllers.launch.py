from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define paths to necessary files
    boxy_description_launch = os.path.join(
        get_package_share_directory('iai_boxy_description'), 'launch', 'upload_boxy.launch.py'
    )
    boxy_start_config = os.path.join(
        get_package_share_directory('iai_boxy_bringup'), 'config', 'boxy_start_config.yaml'
    )

    return LaunchDescription([
        # Argument for conditional upload of default start configuration
        DeclareLaunchArgument(
            'upload_default_start_config',
            default_value='true',
            description='Upload default start configuration to parameter server'
        ),
        
        # Global parameter for simulation time
        Node(
            package='rclcpp',  # For setting the `/use_sim_time` parameter globally
            executable='parameter_bridge',
            name='use_sim_time_param',
            parameters=[{'use_sim_time': False}]
        ),

        # Include the URDF upload launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(boxy_description_launch)
        ),

        # Parameter for joint state publish rate
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='pr2_controller_manager',
            parameters=[{'joint_state_publish_rate': 250.0}]
        ),

        # Conditionally load start configuration parameters
        GroupAction(
            condition=IfCondition(LaunchConfiguration('upload_default_start_config')),
            actions=[
                Node(
                    package='ros2_param_handler',  # Replace with appropriate package if needed
                    executable='rosparam_load',
                    name='loopback_controllers_config',
                    parameters=[boxy_start_config],
                    namespace='loopback_controllers'
                )
            ]
        ),

        # Loopback controller manager node (assuming a replacement is available)
        Node(
            package='ros2_control',  # Adjust if another package for loopback control is used in ROS2
            executable='loopback_controller_manager',
            name='loopback_controllers',
            output='screen',
            parameters=[
                {'dt': 0.004},
                {'damping': 0.0},
                {'mass': 0.01}
            ]
        )
    ])
