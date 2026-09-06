"""Jetson Orin NX competition profile with explicit safety interlocks."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    SetEnvironmentVariable,
)
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_camera = LaunchConfiguration('start_camera')
    start_lidar = LaunchConfiguration('start_lidar')
    start_ultrasonic = LaunchConfiguration('start_ultrasonic')
    start_yolo = LaunchConfiguration('start_yolo')
    start_lane_stack = LaunchConfiguration('start_lane_stack')
    enable_lane_pipeline = LaunchConfiguration('enable_lane_pipeline')
    enable_scene_pipeline = LaunchConfiguration('enable_scene_pipeline')
    enable_drive_control = LaunchConfiguration('enable_drive_control')
    start_rviz = LaunchConfiguration('start_rviz')
    start_active = LaunchConfiguration('start_active')
    start_motor_driver = LaunchConfiguration('start_motor_driver')
    motor_port = LaunchConfiguration('motor_port')

    integrated_launch = PathJoinSubstitution([
        FindPackageShare('mission_cone_drive'),
        'launch',
        'integrated_drive.launch.py',
    ])
    return LaunchDescription([
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument('start_ultrasonic', default_value='false'),
        DeclareLaunchArgument('start_yolo', default_value='true'),
        DeclareLaunchArgument('start_lane_stack', default_value='true'),
        DeclareLaunchArgument('enable_lane_pipeline', default_value='true'),
        DeclareLaunchArgument('enable_scene_pipeline', default_value='true'),
        DeclareLaunchArgument('enable_drive_control', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='false'),
        DeclareLaunchArgument(
            'start_active',
            default_value='false',
            description=(
                'Enables autonomous command generation. Kept false until '
                'the course and emergency stop are ready.'
            ),
        ),
        DeclareLaunchArgument(
            'start_motor_driver',
            default_value='true',
            description=(
                'Starts the integrated VESC driver. Final motor output stays '
                'zero until /start_integrated_drive accepts readiness.'
            ),
        ),
        DeclareLaunchArgument('motor_port', default_value='auto'),
        SetEnvironmentVariable(
            'PYTORCH_CUDA_ALLOC_CONF',
            'expandable_segments:True',
        ),
        LogInfo(
            condition=UnlessCondition(start_active),
            msg='Drive controller starts INACTIVE (safe default).',
        ),
        LogInfo(
            condition=UnlessCondition(start_motor_driver),
            msg='Physical VESC starts DISABLED (safe default).',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(integrated_launch),
            launch_arguments={
                'start_camera': start_camera,
                'start_lidar': start_lidar,
                'start_ultrasonic': start_ultrasonic,
                'start_yolo': start_yolo,
                'start_lane_stack': start_lane_stack,
                'enable_lane_pipeline': enable_lane_pipeline,
                'enable_scene_pipeline': enable_scene_pipeline,
                'enable_drive_control': enable_drive_control,
                'start_rviz': start_rviz,
                'start_active': start_active,
                'start_motor_driver': start_motor_driver,
                'motor_port': motor_port,
                'lane_yolo_device': '0',
                'lane_yolo_fp16': 'true',
                'lane_yolo_runtime_report_path':
                    '/tmp/xycar_runtime/lane_yolo.json',
                'scene_yolo_device': '0',
                'scene_yolo_fp16': 'true',
                'scene_yolo_runtime_report_path':
                    '/tmp/xycar_runtime/scene_yolo.json',
                'lane_cpu_affinity': '0-3',
                'scene_cpu_affinity': '4-7',
                'publish_performance_stats': 'true',
                'resource_stats_output_path':
                    '/tmp/xycar_runtime/resource_stats.csv',
            }.items(),
        ),
    ])
