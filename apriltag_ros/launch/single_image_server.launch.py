"""Launch the AprilTag single-image-detection service server."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('apriltag_ros')

    node_name = LaunchConfiguration('node_name')

    return LaunchDescription([
        DeclareLaunchArgument('node_name', default_value='apriltag_ros_single_image_server_node'),

        Node(
            package='apriltag_ros',
            executable='apriltag_ros_single_image_server_node',
            name=node_name,
            output='screen',
            parameters=[
                os.path.join(pkg_share, 'config', 'settings.yaml'),
                os.path.join(pkg_share, 'config', 'tags.yaml'),
            ],
        ),
    ])
