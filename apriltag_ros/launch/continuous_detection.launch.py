"""Launch the AprilTag continuous detector node against a rectified camera stream."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('apriltag_ros')

    node_name = LaunchConfiguration('node_name')
    camera_name = LaunchConfiguration('camera_name')
    image_topic = LaunchConfiguration('image_topic')

    return LaunchDescription([
        DeclareLaunchArgument('node_name', default_value='apriltag_ros_continuous_node'),
        DeclareLaunchArgument('camera_name', default_value='/camera_rect'),
        DeclareLaunchArgument('image_topic', default_value='image_rect'),

        Node(
            package='apriltag_ros',
            executable='apriltag_ros_continuous_node',
            name=node_name,
            output='screen',
            parameters=[
                os.path.join(pkg_share, 'config', 'settings.yaml'),
                os.path.join(pkg_share, 'config', 'tags.yaml'),
                {'publish_tag_detections_image': True},
            ],
            remappings=[
                ('image_rect', [camera_name, '/', image_topic]),
                ('camera_info', [camera_name, '/camera_info']),
            ],
        ),
    ])
