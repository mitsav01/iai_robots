from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define path to the robot description launch file
    upload_boxy_launch = os.path.join(
        get_package_share_directory('iai_boxy_description'), 'launch', 'upload_boxy.launch.py'
    )

    return LaunchDescription([
        # Global parameter for simulation time
        Node(
            package='rclcpp',
            executable='parameter_bridge',
            name='use_sim_time_param',
            parameters=[{'use_sim_time': False}]
        ),

        # Include the robot description launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(upload_boxy_launch)
        ),

        # TF2 Buffer Server Node
        Node(
            package='tf2_ros',
            executable='buffer_server',
            name='buffer_server',
            output='screen'
        ),

        # Joint State Publisher Node
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{
                'source_list': [
                    'right_arm/joint_states',
                    'left_arm/joint_states',
                    'torso/joint_states',
                    'head/joint_states',
                    'left_arm_gripper/joint_states',
                    'right_arm_gripper/joint_states',
                    'neck/joint_states'
                ],
                'zeros': {
                    'left_arm_0_joint': 0.47553,
                    'left_arm_1_joint': 2.09769,
                    'left_arm_2_joint': 2.97525,
                    'left_arm_3_joint': 1.5025,
                    'left_arm_4_joint': -2.96865,
                    'left_arm_5_joint': 0.394601,
                    'left_arm_6_joint': 0.478342,
                    'right_arm_0_joint': -0.31,
                    'right_arm_1_joint': -1.43,
                    'right_arm_2_joint': 1.16,
                    'right_arm_3_joint': 1.52,
                    'right_arm_4_joint': -1.16,
                    'right_arm_5_joint': -1.17,
                    'right_arm_6_joint': 0.1
                },
                'rate': 200,
                'use_gui': True
            }]
        ),
    ])
