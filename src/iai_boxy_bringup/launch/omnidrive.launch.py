from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='omni_ethercat',
            executable='omni_ethercat',
            name='omnidrive',
            output='screen',
            parameters=[
                {'speed': 1.0},
                {'acceleration': 2.0},
                {'frame_id': '/odom'},
                {'child_frame_id': '/base_footprint'},
                {'tf_frequency': 20}
            ]
        )
    ])
