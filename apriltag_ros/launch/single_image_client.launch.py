"""Launch the AprilTag single-image-detection service client.

Calls the running 'single_image_tag_detection' service (see
single_image_server.launch.py) once, analyzes the image at image_load_path,
and writes an annotated copy to image_save_path.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('apriltag_ros')

    node_name = LaunchConfiguration('node_name')

    return LaunchDescription([
        DeclareLaunchArgument('node_name', default_value='apriltag_ros_single_image_client_node'),
        DeclareLaunchArgument('image_load_path',
                               description='Full path of the image to load for analysis'),
        DeclareLaunchArgument('image_save_path',
                               description='Full path to save the annotated detections image'),
        DeclareLaunchArgument('fx', default_value='652.7934615847107'),
        DeclareLaunchArgument('fy', default_value='653.9480389077635'),
        DeclareLaunchArgument('cx', default_value='307.1288710375904'),
        DeclareLaunchArgument('cy', default_value='258.7823279214385'),

        Node(
            package='apriltag_ros',
            executable='apriltag_ros_single_image_client_node',
            name=node_name,
            output='screen',
            parameters=[
                os.path.join(pkg_share, 'config', 'settings.yaml'),
                os.path.join(pkg_share, 'config', 'tags.yaml'),
                {
                    'image_load_path': LaunchConfiguration('image_load_path'),
                    'image_save_path': LaunchConfiguration('image_save_path'),
                    'fx': ParameterValue(LaunchConfiguration('fx'), value_type=float),
                    'fy': ParameterValue(LaunchConfiguration('fy'), value_type=float),
                    'cx': ParameterValue(LaunchConfiguration('cx'), value_type=float),
                    'cy': ParameterValue(LaunchConfiguration('cy'), value_type=float),
                },
            ],
        ),
    ])
