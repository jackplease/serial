import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    serial_node = Node(
        package='serial',
        executable='serial_node',
        namespace='',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([serial_node])
