import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('teleops'),
        'config',
        'params.yaml',
    )

    return LaunchDescription([
        Node(
            package='teleops',
            executable='avr_interface_node',
            name='avr_interface_node',
            parameters=[config],
            output='screen',
        ),
    ])
