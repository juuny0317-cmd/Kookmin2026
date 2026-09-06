from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sync_slop', default_value='0.2'),
        DeclareLaunchArgument('sync_queue_size', default_value='10'),
        DeclareLaunchArgument('debug_view', default_value='false'),
        DeclareLaunchArgument('min_color_ratio', default_value='0.025'),
        DeclareLaunchArgument('stable_frames', default_value='2'),
        Node(
            package='cam',
            executable='traffic_light_node',
            name='traffic_light_node',
            output='screen',
            parameters=[{
                'image_topic': '/image_raw',
                'detection_topic': '/yolo_detections',
                'sync_slop': LaunchConfiguration('sync_slop'),
                'sync_queue_size': LaunchConfiguration('sync_queue_size'),
                'debug_view': LaunchConfiguration('debug_view'),
                'min_color_ratio': LaunchConfiguration('min_color_ratio'),
                'stable_frames': LaunchConfiguration('stable_frames'),
            }],
        ),
    ])
