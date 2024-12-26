from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare arguments
    kinematics_config_left = DeclareLaunchArgument(
        'kinematics_config_left',
        default_value=os.path.join(
            get_package_share_directory('iai_dualarm_description'),
            'config/ur5_left_arm_calibration.yaml'
        )
    )
    kinematics_config_right = DeclareLaunchArgument(
        'kinematics_config_right',
        default_value=os.path.join(
            get_package_share_directory('iai_dualarm_description'),
            'config/ur5_right_arm_calibration.yaml'
        )
    )
    transmission_hw_interface = DeclareLaunchArgument(
        'transmission_hw_interface',
        default_value='hardware_interface/PositionJointInterface'
    )
    urdf_name = DeclareLaunchArgument('urdf_name', default_value='dualarm_ur5s_one_gripper.urdf.xacro')
    urdf_path = LaunchConfiguration('urdf_path', default=os.path.join(
        get_package_share_directory('iai_dualarm_description'),
        'robots', LaunchConfiguration('urdf_name')
    ))

    # Static transforms
    map_world_transformer = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_world_transformer",
        arguments=["0", "0", "0", "0", "0", "0", "1", "map", "world"]
    )
    map_world_transformer2 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_world_transformer2",
        arguments=["0", "0", "0", "0", "0", "0", "1", "map", "ur5dualarm/world"]
    )

    # Robot State Publisher for URDF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            'robot_description': os.path.join(
                get_package_share_directory('iai_dualarm_description'),
                'robots', 'dualarm_ur5s_one_gripper.urdf.xacro'
            )
        }]
    )

    # Left arm UR5 bringup
    left_arm_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ur_robot_driver'), 'launch', 'ur5_bringup.launch.py')
        ),
        launch_arguments={
            'robot_ip': '192.168.101.1',
            'tf_prefix': 'left_',
            'controller_config_file': os.path.join(
                get_package_share_directory('iai_dualarm_description'),
                'config', 'ur5_real_left_arm_control.yaml'
            ),
            'kinematics_config': LaunchConfiguration('kinematics_config_left'),
            'robot_description_file': os.path.join(
                get_package_share_directory('iai_dualarm_description'),
                'launch', 'upload_dualarm_ur5s_one_gripper.launch.py'
            ),
            'controllers': 'joint_state_controller_left joint_group_vel_controller_left',
            'stopped_controllers': 'pos_joint_traj_controller_left',
            'reverse_port': '50008',
            'script_sender_port': '50011',
            'trajectory_port': '50010',
            'script_command_port': '50012'
        }.items()
    )

    # Right arm UR5 bringup
    right_arm_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ur_robot_driver'), 'launch', 'ur5_bringup.launch.py')
        ),
        launch_arguments={
            'robot_ip': '192.168.101.171',
            'tf_prefix': 'right_',
            'controller_config_file': os.path.join(
                get_package_share_directory('iai_dualarm_description'),
                'config', 'ur5_real_right_arm_control.yaml'
            ),
            'kinematics_config': LaunchConfiguration('kinematics_config_right'),
            'robot_description_file': os.path.join(
                get_package_share_directory('iai_dualarm_description'),
                'launch', 'upload_dualarm_ur5s_one_gripper.launch.py'
            ),
            'controllers': 'joint_state_controller_right joint_group_vel_controller_right',
            'stopped_controllers': 'pos_joint_traj_controller_right',
            'reverse_port': '50000',
            'script_sender_port': '50002',
            'trajectory_port': '50001',
            'script_command_port': '50003'
        }.items()
    )

    # Left gripper configuration
    left_gripper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('wsg_50_driver'), 'launch', 'wsg_50_tcp.launch.py')
        ),
        launch_arguments={
            'gripper_name': 'left_gripper',
            'finger_joint_name': 'left_gripper_joint',
            'ip': '192.168.102.63'
        }.items()
    )

    # Robot State Publisher for the left gripper
    left_gripper_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="rob_st_pub",
        namespace="left_gripper",
        remappings=[("joint_states", "wsg_50_driver/joint_states")]
    )

    # Joint State Publisher
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{
            'source_list': ['/left_arm/joint_states', '/right_arm/joint_states', '/left_gripper/wsg_50_driver/joint_states'],
            'rate': 120,
            'use_gui': False
        }]
    )

    # RViz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=['-d', os.path.join(get_package_share_directory('iai_dualarm_description'), 'urdf.rviz')]
    )

    # Launch Description
    return LaunchDescription([
        kinematics_config_left,
        kinematics_config_right,
        transmission_hw_interface,
        urdf_name,
        map_world_transformer,
        map_world_transformer2,
        robot_state_publisher,
        left_arm_bringup,
        right_arm_bringup,
        left_gripper,
        left_gripper_state_publisher,
        joint_state_publisher,
        rviz_node
    ])
