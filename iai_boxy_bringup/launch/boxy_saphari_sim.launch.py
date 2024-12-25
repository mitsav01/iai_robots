from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define paths to necessary files
    iai_boxy_description_launch = os.path.join(
        get_package_share_directory('iai_boxy_description'), 'launch', 'upload_boxy.launch.py'
    )
    dlr_action_bridge_launch = os.path.join(
        get_package_share_directory('dlr_action_bridge'), 'launch', 'single_arm.launch.py'
    )
    iai_boxy_bringup_launch = os.path.join(
        get_package_share_directory('iai_boxy_bringup'), 'launch', 'boxy_arm_ik.launch.py'
    )

    return LaunchDescription([
        # Include the iai_boxy_description launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(iai_boxy_description_launch)
        ),

        # Include the dlr_action_bridge single_arm.launch with arguments
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(dlr_action_bridge_launch),
            launch_arguments={
                'action_name': 'right_arm',
                'node_name': 'right_arm',
                'joint_prefix': 'right_',
                'joint_states_rate': '40'
            }.items()
        ),

        # Robot State Publisher Node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='log'
        ),

        # Joint State Publisher Node
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{
                'source_list': ['right_arm/joint_states'],
                'zeros': {
                    'triangle_base_joint': -0.3,
                    'left_arm_0_joint': 0.64,
                    'left_arm_1_joint': 2.0
                },
                'rate': 20
            }]
        ),

        # Include the boxy_arm_ik launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(iai_boxy_bringup_launch)
        ),
    ])
