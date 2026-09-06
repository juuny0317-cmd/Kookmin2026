from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model_filename',
            default_value='center_line_yolov10n_320_best.pt'),
        DeclareLaunchArgument('cone_conf_threshold', default_value='0.20'),
        Node(
            package='cam',
            executable='yolo_node',
            name='old_yolo_node',
            parameters=[{
                'image_topic_name': '/image_raw',
                'show_visualization': True,
                'show_labels': True,
                'show_conf': True,
                'model_filename': LaunchConfiguration('model_filename'),
                'cone_conf_threshold': LaunchConfiguration('cone_conf_threshold'),
            }],
        ),
        Node(
            package='cam',
            executable='centerlane_tracer',
            name='centerlane_tracer',
            parameters=[{
                'image_topic': '/image_raw',
                'lane_detections_topic': '/yolo_detections',
                'enable_scene_detection_cache': False,
            }],
        ),
        Node(
            package='cam',
            executable='traffic_light_node',
            name='traffic_light_node',
            parameters=[{
                'image_topic': '/image_raw',
                'detection_topic': '/yolo_detections',
                'sync_slop': 0.2,
                'sync_queue_size': 10,
                'debug_view': False,
            }],
        ),
    ])
