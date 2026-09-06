import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rotation_deg = LaunchConfiguration('rotation_deg')
    motor_topic = LaunchConfiguration('motor_topic')
    min_speed = LaunchConfiguration('min_speed')
    max_speed = LaunchConfiguration('max_speed')
    start_rviz = LaunchConfiguration('start_rviz')
    use_rviz_markers = LaunchConfiguration('use_rviz_markers')

    max_range_m = LaunchConfiguration('max_range_m')
    x_max_m = LaunchConfiguration('x_max_m')
    y_abs_max_m = LaunchConfiguration('y_abs_max_m')
    gap_min_width_m = LaunchConfiguration('gap_min_width_m')
    gap_max_width_m = LaunchConfiguration('gap_max_width_m')
    gap_expected_width_m = LaunchConfiguration('gap_expected_width_m')
    cone_spacing_m = LaunchConfiguration('cone_spacing_m')
    one_side_half_width_m = LaunchConfiguration('one_side_half_width_m')
    min_lookahead_m = LaunchConfiguration('min_lookahead_m')
    max_lookahead_m = LaunchConfiguration('max_lookahead_m')
    lookahead_scale = LaunchConfiguration('lookahead_scale')

    rviz_config = os.path.join(
        get_package_share_directory('mission_cone_drive'),
        'rviz',
        'cone_drive.rviz',
    )

    return LaunchDescription([
        DeclareLaunchArgument('rotation_deg', default_value='0.0'),
        DeclareLaunchArgument('motor_topic', default_value='/xycar_motor'),
        DeclareLaunchArgument('min_speed', default_value='0.0'),
        DeclareLaunchArgument('max_speed', default_value='0.0'),
        DeclareLaunchArgument('start_rviz', default_value='false'),
        DeclareLaunchArgument('use_rviz_markers', default_value='true'),
        DeclareLaunchArgument('max_range_m', default_value='1.80'),
        DeclareLaunchArgument('x_max_m', default_value='1.80'),
        DeclareLaunchArgument('y_abs_max_m', default_value='0.80'),
        DeclareLaunchArgument('gap_min_width_m', default_value='0.50'),
        DeclareLaunchArgument('gap_max_width_m', default_value='1.00'),
        DeclareLaunchArgument('gap_expected_width_m', default_value='0.80'),
        DeclareLaunchArgument('cone_spacing_m', default_value='0.30'),
        DeclareLaunchArgument('one_side_half_width_m', default_value='0.40'),
        DeclareLaunchArgument('min_lookahead_m', default_value='0.30'),
        DeclareLaunchArgument('max_lookahead_m', default_value='0.60'),
        DeclareLaunchArgument('lookahead_scale', default_value='0.30'),
        DeclareLaunchArgument('cluster_min_points', default_value='2'),
        DeclareLaunchArgument(
            'cluster_min_chord_m', default_value='0.02'),
        DeclareLaunchArgument(
            'cluster_max_radius_m', default_value='0.12'),
        DeclareLaunchArgument(
            'cluster_max_chord_m', default_value='0.24'),
        DeclareLaunchArgument(
            'association_distance_m', default_value='0.22'),
        DeclareLaunchArgument('min_track_hits', default_value='2'),
        DeclareLaunchArgument('max_track_misses', default_value='2'),
        DeclareLaunchArgument('min_entry_pairs', default_value='3'),
        DeclareLaunchArgument('min_pair_span_m', default_value='0.18'),
        DeclareLaunchArgument(
            'max_entry_width_spread_m', default_value='0.20'),
        DeclareLaunchArgument(
            'boundary_tolerance_m', default_value='0.18'),
        DeclareLaunchArgument('min_one_side_points', default_value='2'),
        DeclareLaunchArgument(
            'two_point_support_confirm_frames', default_value='2'),
        DeclareLaunchArgument(
            'established_hold_frames', default_value='10'),
        Node(
            package='mission_cone_drive',
            executable='scan_rotator',
            name='scan_rotator_node',
            output='screen',
            parameters=[{
                'input_topic': '/scan',
                'output_topic': '/scan_rotated',
                'rotation_deg': rotation_deg,
            }],
        ),
        Node(
            package='mission_cone_drive',
            executable='lidar_cone_filter_node',
            name='lidar_cone_filter_node',
            output='screen',
            parameters=[{
                'scan_topic': '/scan_rotated',
                'raw_clusters_topic': '/clusters_raw',
                'output_clusters_topic': '/clusters',
                'max_range_m': max_range_m,
                'x_max_m': x_max_m,
                'y_abs_max_m': y_abs_max_m,
                'cluster_min_points': LaunchConfiguration(
                    'cluster_min_points'),
                'cluster_min_chord_m': LaunchConfiguration(
                    'cluster_min_chord_m'),
                'cluster_max_radius_m': LaunchConfiguration(
                    'cluster_max_radius_m'),
                'cluster_max_chord_m': LaunchConfiguration(
                    'cluster_max_chord_m'),
                'gap_min_width_m': gap_min_width_m,
                'gap_max_width_m': gap_max_width_m,
                'gap_expected_width_m': gap_expected_width_m,
                'cone_spacing_m': cone_spacing_m,
                'association_distance_m': LaunchConfiguration(
                    'association_distance_m'),
                'min_track_hits': LaunchConfiguration('min_track_hits'),
                'max_track_misses': LaunchConfiguration(
                    'max_track_misses'),
                'min_entry_pairs': LaunchConfiguration('min_entry_pairs'),
                'min_pair_span_m': LaunchConfiguration('min_pair_span_m'),
                'max_entry_width_spread_m': LaunchConfiguration(
                    'max_entry_width_spread_m'),
                'one_side_half_width_m': one_side_half_width_m,
                'boundary_tolerance_m': LaunchConfiguration(
                    'boundary_tolerance_m'),
                'min_one_side_points': LaunchConfiguration(
                    'min_one_side_points'),
                'two_point_support_confirm_frames': LaunchConfiguration(
                    'two_point_support_confirm_frames'),
                'established_hold_frames': LaunchConfiguration(
                    'established_hold_frames'),
            }],
        ),
        Node(
            package='mission_cone_drive',
            executable='path_planning_node',
            name='path_planning_node',
            output='screen',
            parameters=[{
                'clusters_topic': '/clusters',
                'path_topic': '/path',
                'require_green_signal': False,
                'use_precomputed_corridor': True,
                'x_max_m': x_max_m,
                'y_abs_max_m': y_abs_max_m,
                'gap_min_width_m': gap_min_width_m,
                'gap_max_width_m': gap_max_width_m,
                'gap_expected_width_m': gap_expected_width_m,
                'cone_spacing_m': cone_spacing_m,
                'one_side_half_width_m': one_side_half_width_m,
                'min_midpoints_for_path': 2,
                'min_path_span_m': 0.18,
                'path_lost_keep_frames': 3,
                'path_smoothing_alpha': 0.75,
            }],
        ),
        Node(
            package='mission_cone_drive',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            output='screen',
            parameters=[{
                'path_topic': '/path',
                'motor_topic': motor_topic,
                'require_green_signal': False,
                'min_speed': min_speed,
                'max_speed': max_speed,
                'min_lookahead_m': min_lookahead_m,
                'max_lookahead_m': max_lookahead_m,
                'lookahead_scale': lookahead_scale,
                'min_path_points': 2,
                'command_publish_rate_hz': 10.0,
            }],
        ),
        Node(
            package='mission_cone_drive',
            executable='rviz_visualizer_node',
            name='rviz_visualizer_node',
            output='screen',
            condition=IfCondition(use_rviz_markers),
            parameters=[{'motor_topic': motor_topic}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='lidar_cone_drive_rviz',
            arguments=['-d', rviz_config],
            condition=IfCondition(start_rviz),
        ),
    ])
