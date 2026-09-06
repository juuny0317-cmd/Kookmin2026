from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cam',
            executable='lane_detector',
            name='lane_detector_node',
            parameters=[{
                'image_topic': '/image_raw',
                'lane_detections_topic': '/yolo_detections',
                'scene_detections_topic': '/yolo_detections',
            }],
        ),
        Node(
            package='cam',
            executable='integrated_stanley_controller',
            name='stanley_node',
        ),
        Node(
            package='cam',
            executable='yolo_node',
            name='yolo_node',
        ),
        Node(
            package='cam',
            executable='ultra_node',
            name='ultra_node',
        ),
        Node(
            package='cam',
            executable='target_lane_planner',
            name='target_lane_planner',
            parameters=[{
                'image_topic': '/image_raw',
                'detections_topic': '/yolo_detections',
                'curve_source_width': 640,
                'curve_source_height': 480,
            }],
        ),
        Node(
            package='cam',
            executable='centerlane_tracer',
            name='centerlane_tracer',
            parameters=[{
                'image_topic': '/image_raw',
                'lane_detections_topic': '/yolo_detections',
                'scene_detections_topic': '/yolo_detections',
            }],
        ),
        Node(
        package='mission_cone_drive',
        executable='checkerboard_detector',
        name='checkerboard_node'
    ),
    ])
