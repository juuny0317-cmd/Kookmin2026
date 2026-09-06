"""Launch usb_cam with passive T0..T7 profiling enabled."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    usb_cam_share = get_package_share_directory('usb_cam')
    camera_params = os.path.join(usb_cam_share, 'config', 'params.yaml')
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='xycar_cam',
            output='screen',
            parameters=[
                camera_params,
                {
                    'publish_camera_timing': True,
                    'camera_timing_topic': '/camera_timing',
                },
            ],
        ),
        Node(
            package='cam',
            executable='camera_timing_probe',
            name='camera_timing_probe',
            output='screen',
        ),
    ])
