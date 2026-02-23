import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('teleops_operator')
    params_file = os.path.join(pkg_share, 'config', 'joy_params.yaml')

    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[params_file],
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_twist_joy_node',
            name='teleop_twist_joy_node',
            parameters=[params_file],
        ),
    ])
