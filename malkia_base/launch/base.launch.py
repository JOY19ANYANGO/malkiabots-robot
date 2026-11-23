import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('malkia_base'), 'config', 'base.yaml')
    return LaunchDescription([
        Node(package='malkia_base', executable='base_node', name='base_node',
             output='screen', parameters=[cfg]),
    ])
