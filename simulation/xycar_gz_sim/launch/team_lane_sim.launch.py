"""Run the supplied team lane-driving code on the existing Gazebo track."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    xycar_share = get_package_share_directory('xycar_gz_sim')
    config = os.path.join(xycar_share, 'config', 'xycar.yaml')
    default_world = os.path.join(
        xycar_share, 'worlds', 'kookmin_track_from_dxf.world.sdf')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument(
            'world',
            default_value=default_world),
        # Start on the long bottom straight in the source centreline order so
        # the controller sees broad outer-loop bends before the tight S-turns.
        DeclareLaunchArgument('spawn_x', default_value='4.0'),
        DeclareLaunchArgument('spawn_y', default_value='-3.468'),
        DeclareLaunchArgument('spawn_z', default_value='0.02'),
        DeclareLaunchArgument('spawn_yaw', default_value='3.141592654'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('start_drive', default_value='true'),
        DeclareLaunchArgument('debug_view', default_value='false'),
        # The map has continuous white road edges and a dashed yellow centre.
        # Use both white edges first, while retaining the team's yellow-line
        # fallback when only the inside of a tight bend remains visible.
        DeclareLaunchArgument('lane_mode', default_value='road'),
        DeclareLaunchArgument('straight_speed', default_value='4.0'),
        DeclareLaunchArgument('curve_speed', default_value='4.0'),
        DeclareLaunchArgument('s_curve_speed', default_value='4.0'),
        # Gazebo's camera shows useful near-field markings down to y=479.
        # The real-camera default y=385 clips the nearest dash on this model.
        DeclareLaunchArgument('warp_bottom_y', default_value='479.0'),
        DeclareLaunchArgument('sliding_margin_px', default_value='100'),
        DeclareLaunchArgument('sliding_minpix', default_value='5'),
        DeclareLaunchArgument('min_center_line_points', default_value='2'),
        DeclareLaunchArgument('path_fit_min_points', default_value='2'),
        DeclareLaunchArgument('mask_min_component_area_px', default_value='10'),
        DeclareLaunchArgument('histogram_y_start_ratio', default_value='0.50'),
        # A tight bend can move the visible yellow line by more than the real
        # camera profile's 120 px history gate.  Allow full-frame reacquisition
        # in simulation instead of latching the old peak and stopping.
        DeclareLaunchArgument('yellow_center_switch_gate_px', default_value='640'),
        DeclareLaunchArgument('yellow_center_prev_weight', default_value='0.20'),
        DeclareLaunchArgument('yellow_center_image_weight', default_value='0.80'),
        DeclareLaunchArgument('lost_stop_frames', default_value='12'),
        DeclareLaunchArgument('max_reuse_frames', default_value='15'),
        # Gazebo markings are geometrically sharp and produced larger
        # frame-to-frame derivative spikes than the real-camera recording.
        DeclareLaunchArgument('pid_kp', default_value='0.12'),
        DeclareLaunchArgument('pid_kd', default_value='0.02'),
        DeclareLaunchArgument('heading_kp', default_value='8.0'),
        DeclareLaunchArgument('preview_heading_kp', default_value='4.0'),
        DeclareLaunchArgument('far_heading_kp', default_value='4.0'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(xycar_share, 'launch', 'xycar.launch.py')),
            launch_arguments={
                'use_sim': 'true',
                'gui': LaunchConfiguration('gui'),
                'world': LaunchConfiguration('world'),
                'spawn_x': LaunchConfiguration('spawn_x'),
                'spawn_y': LaunchConfiguration('spawn_y'),
                'spawn_z': LaunchConfiguration('spawn_z'),
                'spawn_yaw': LaunchConfiguration('spawn_yaw'),
            }.items(),
        ),
        Node(
            package='xycar_gz_sim',
            executable='team_motor_adapter',
            name='team_motor_adapter',
            output='screen',
            parameters=[config, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='track_drive',
            executable='lane_drive',
            name='team_lane_drive',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_drive')),
            parameters=[{
                'use_sim_time': use_sim_time,
                'image_topic': '/image_raw',
                'motor_topic': '/team/xycar_motor_cmd',
                'debug_view': ParameterValue(
                    LaunchConfiguration('debug_view'), value_type=bool),
                'lane_mode': LaunchConfiguration('lane_mode'),
                'straight_speed': ParameterValue(
                    LaunchConfiguration('straight_speed'), value_type=float),
                'curve_speed': ParameterValue(
                    LaunchConfiguration('curve_speed'), value_type=float),
                's_curve_speed': ParameterValue(
                    LaunchConfiguration('s_curve_speed'), value_type=float),
                'warp_bottom_y': ParameterValue(
                    LaunchConfiguration('warp_bottom_y'), value_type=float),
                'sliding_margin_px': ParameterValue(
                    LaunchConfiguration('sliding_margin_px'), value_type=int),
                'sliding_minpix': ParameterValue(
                    LaunchConfiguration('sliding_minpix'), value_type=int),
                'min_center_line_points': ParameterValue(
                    LaunchConfiguration('min_center_line_points'), value_type=int),
                'path_fit_min_points': ParameterValue(
                    LaunchConfiguration('path_fit_min_points'), value_type=int),
                'mask_min_component_area_px': ParameterValue(
                    LaunchConfiguration('mask_min_component_area_px'), value_type=int),
                'histogram_y_start_ratio': ParameterValue(
                    LaunchConfiguration('histogram_y_start_ratio'), value_type=float),
                'yellow_center_switch_gate_px': ParameterValue(
                    LaunchConfiguration('yellow_center_switch_gate_px'), value_type=int),
                'yellow_center_prev_weight': ParameterValue(
                    LaunchConfiguration('yellow_center_prev_weight'), value_type=float),
                'yellow_center_image_weight': ParameterValue(
                    LaunchConfiguration('yellow_center_image_weight'), value_type=float),
                'lost_stop_frames': ParameterValue(
                    LaunchConfiguration('lost_stop_frames'), value_type=int),
                'max_reuse_frames': ParameterValue(
                    LaunchConfiguration('max_reuse_frames'), value_type=int),
                'pid_kp': ParameterValue(
                    LaunchConfiguration('pid_kp'), value_type=float),
                'pid_kd': ParameterValue(
                    LaunchConfiguration('pid_kd'), value_type=float),
                'heading_kp': ParameterValue(
                    LaunchConfiguration('heading_kp'), value_type=float),
                'preview_heading_kp': ParameterValue(
                    LaunchConfiguration('preview_heading_kp'), value_type=float),
                'far_heading_kp': ParameterValue(
                    LaunchConfiguration('far_heading_kp'), value_type=float),
            }],
        ),
    ])
