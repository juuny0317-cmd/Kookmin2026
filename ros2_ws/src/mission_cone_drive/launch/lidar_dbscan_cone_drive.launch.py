import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_lidar = LaunchConfiguration('start_lidar')
    start_rviz = LaunchConfiguration('start_rviz')
    motor_topic = LaunchConfiguration('motor_topic')
    rotation_deg = LaunchConfiguration('rotation_deg')
    min_speed = LaunchConfiguration('min_speed')
    max_speed = LaunchConfiguration('max_speed')
    rear_axle_offset_m = LaunchConfiguration('rear_axle_offset_m')
    cone_controller = LaunchConfiguration('cone_controller')

    lidar_launch = PathJoinSubstitution([
        FindPackageShare('xycar_lidar'),
        'launch',
        'xycar_lidar.launch.py',
    ])
    rviz_config = os.path.join(
        get_package_share_directory('mission_cone_drive'),
        'rviz',
        'cone_drive.rviz',
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('motor_topic', default_value='/xycar_motor'),
        DeclareLaunchArgument('rotation_deg', default_value='0.0'),
        DeclareLaunchArgument(
            'cone_controller',
            default_value='pure_pursuit',
            choices=['pure_pursuit', 'stanley'],
        ),
        DeclareLaunchArgument('min_speed', default_value='0.0'),
        DeclareLaunchArgument('max_speed', default_value='0.0'),
        DeclareLaunchArgument('min_lookahead_m', default_value='0.70'),
        DeclareLaunchArgument('max_lookahead_m', default_value='2.50'),
        DeclareLaunchArgument('lookahead_scale', default_value='0.25'),
        DeclareLaunchArgument('rear_axle_offset_m', default_value='0.42'),
        DeclareLaunchArgument('min_range_m', default_value='0.18'),
        DeclareLaunchArgument('max_range_m', default_value='1.50'),
        DeclareLaunchArgument('dbscan_eps_m', default_value='0.04'),
        DeclareLaunchArgument('dbscan_min_samples', default_value='3'),
        DeclareLaunchArgument(
            'max_cone_diameter_m', default_value='0.30'),
        DeclareLaunchArgument('angle_bin_deg', default_value='0.0'),
        DeclareLaunchArgument(
            'min_cluster_separation_m', default_value='0.15'),
        DeclareLaunchArgument('seed_max_range_m', default_value='0.80'),
        DeclareLaunchArgument('grow_distance_m', default_value='0.50'),
        DeclareLaunchArgument('max_pair_distance_m', default_value='1.30'),
        DeclareLaunchArgument(
            'virtual_half_width_m', default_value='0.50'),
        DeclareLaunchArgument(
            'max_path_heading_jump_deg', default_value='30.0'),
        DeclareLaunchArgument(
            'max_path_lateral_jump_m', default_value='0.12'),
        DeclareLaunchArgument(
            'max_path_curvature_per_m', default_value='6.0'),
        DeclareLaunchArgument(
            'path_lost_keep_frames', default_value='4'),
        DeclareLaunchArgument('path_lost_keep_s', default_value='0.45'),
        DeclareLaunchArgument(
            'motion_compensation_enabled', default_value='true'),
        DeclareLaunchArgument(
            'prediction_min_forward_m', default_value='0.90'),
        DeclareLaunchArgument(
            'min_path_horizon_m', default_value='0.90'),
        DeclareLaunchArgument(
            'prediction_max_extension_heading_deg', default_value='35.0'),
        DeclareLaunchArgument(
            'one_side_measurement_alpha', default_value='0.25'),
        DeclareLaunchArgument(
            'one_side_max_lateral_correction_m', default_value='0.20'),
        DeclareLaunchArgument(
            'one_side_max_heading_correction_deg', default_value='12.0'),
        DeclareLaunchArgument(
            'one_side_max_age_since_paired_s', default_value='0.45'),
        DeclareLaunchArgument(
            'steering_max_angle_step', default_value='12.0'),
        DeclareLaunchArgument(
            'steering_smoothing_alpha', default_value='0.65'),
        DeclareLaunchArgument(
            'stanley_control_x_m', default_value='0.33'),
        DeclareLaunchArgument(
            'stanley_heading_window_m', default_value='0.30'),
        DeclareLaunchArgument(
            'stanley_cross_track_gain', default_value='1.20'),
        DeclareLaunchArgument(
            'stanley_heading_gain', default_value='1.00'),
        DeclareLaunchArgument(
            'stanley_softening_speed_mps', default_value='0.50'),
        DeclareLaunchArgument(
            'stanley_max_angle_step', default_value='15.0'),
        DeclareLaunchArgument(
            'stanley_smoothing_alpha', default_value='0.65'),
        DeclareLaunchArgument(
            'stanley_path_timeout_s', default_value='0.50'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch),
            condition=IfCondition(start_lidar),
        ),
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
            executable='lidar_dbscan_preprocessing_node',
            name='lidar_dbscan_preprocessing_node',
            output='screen',
            parameters=[{
                'require_green_signal': False,
                'min_range_m': LaunchConfiguration('min_range_m'),
                'max_range_m': LaunchConfiguration('max_range_m'),
                'dbscan_eps_m': LaunchConfiguration('dbscan_eps_m'),
                'dbscan_min_samples': LaunchConfiguration(
                    'dbscan_min_samples'),
                'max_cone_diameter_m': LaunchConfiguration(
                    'max_cone_diameter_m'),
                'angle_bin_deg': LaunchConfiguration('angle_bin_deg'),
                'min_cluster_separation_m': LaunchConfiguration(
                    'min_cluster_separation_m'),
            }],
        ),
        Node(
            package='mission_cone_drive',
            executable='lidar_dbscan_path_planning_node',
            name='lidar_dbscan_path_planning_node',
            output='screen',
            parameters=[{
                'require_green_signal': False,
                'left_seed_max_range_m': LaunchConfiguration(
                    'seed_max_range_m'),
                'right_seed_max_range_m': LaunchConfiguration(
                    'seed_max_range_m'),
                'grow_distance_m': LaunchConfiguration('grow_distance_m'),
                'max_pair_distance_m': LaunchConfiguration(
                    'max_pair_distance_m'),
                'virtual_half_width_m': LaunchConfiguration(
                    'virtual_half_width_m'),
                'rear_axle_offset_m': rear_axle_offset_m,
                'max_path_heading_jump_deg': LaunchConfiguration(
                    'max_path_heading_jump_deg'),
                'max_path_lateral_jump_m': LaunchConfiguration(
                    'max_path_lateral_jump_m'),
                'max_path_curvature_per_m': LaunchConfiguration(
                    'max_path_curvature_per_m'),
                'path_lost_keep_frames': LaunchConfiguration(
                    'path_lost_keep_frames'),
                'path_lost_keep_s': LaunchConfiguration(
                    'path_lost_keep_s'),
                'motion_compensation_enabled': LaunchConfiguration(
                    'motion_compensation_enabled'),
                'prediction_min_forward_m': LaunchConfiguration(
                    'prediction_min_forward_m'),
                'min_path_horizon_m': LaunchConfiguration(
                    'min_path_horizon_m'),
                'prediction_max_extension_heading_deg': LaunchConfiguration(
                    'prediction_max_extension_heading_deg'),
                'one_side_measurement_alpha': LaunchConfiguration(
                    'one_side_measurement_alpha'),
                'one_side_max_lateral_correction_m': LaunchConfiguration(
                    'one_side_max_lateral_correction_m'),
                'one_side_max_heading_correction_deg': LaunchConfiguration(
                    'one_side_max_heading_correction_deg'),
                'one_side_max_age_since_paired_s': LaunchConfiguration(
                    'one_side_max_age_since_paired_s'),
            }],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_dbscan_rear_axle_tf',
            output='screen',
            arguments=[
                '--x',
                PythonExpression(['-1.0 * ', rear_axle_offset_m]),
                '--y', '0.0',
                '--z', '0.0',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'laser_frame',
                '--child-frame-id', 'rear_axle',
            ],
        ),
        Node(
            package='mission_cone_drive',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            output='screen',
            condition=IfCondition(PythonExpression([
                "'", cone_controller, "' == 'pure_pursuit'",
            ])),
            parameters=[{
                'require_green_signal': False,
                'path_topic': '/path',
                'motor_topic': motor_topic,
                'control_x_offset_m': 0.0,
                'min_speed': min_speed,
                'max_speed': max_speed,
                'min_lookahead_m': LaunchConfiguration(
                    'min_lookahead_m'),
                'max_lookahead_m': LaunchConfiguration(
                    'max_lookahead_m'),
                'lookahead_scale': LaunchConfiguration('lookahead_scale'),
                'steering_max_angle_step': LaunchConfiguration(
                    'steering_max_angle_step'),
                'steering_smoothing_alpha': LaunchConfiguration(
                    'steering_smoothing_alpha'),
                'min_path_points': 1,
                'command_publish_rate_hz': 10.0,
            }],
        ),
        Node(
            package='mission_cone_drive',
            executable='cone_stanley_node',
            name='cone_stanley_node',
            output='screen',
            condition=IfCondition(PythonExpression([
                "'", cone_controller, "' == 'stanley'",
            ])),
            parameters=[{
                'require_green_signal': False,
                'path_topic': '/path',
                'motor_topic': motor_topic,
                'min_speed': min_speed,
                'max_speed': max_speed,
                'control_x_m': LaunchConfiguration(
                    'stanley_control_x_m'),
                'heading_window_m': LaunchConfiguration(
                    'stanley_heading_window_m'),
                'cross_track_gain': LaunchConfiguration(
                    'stanley_cross_track_gain'),
                'heading_gain': LaunchConfiguration(
                    'stanley_heading_gain'),
                'softening_speed_mps': LaunchConfiguration(
                    'stanley_softening_speed_mps'),
                'max_angle_step': LaunchConfiguration(
                    'stanley_max_angle_step'),
                'smoothing_alpha': LaunchConfiguration(
                    'stanley_smoothing_alpha'),
                'path_timeout_s': LaunchConfiguration(
                    'stanley_path_timeout_s'),
                'min_path_points': 2,
                'command_publish_rate_hz': 10.0,
            }],
        ),
        Node(
            package='mission_cone_drive',
            executable='rviz_visualizer_node',
            name='rviz_visualizer_node',
            output='screen',
            condition=IfCondition(start_rviz),
            parameters=[{'motor_topic': motor_topic}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='lidar_dbscan_cone_drive_rviz',
            arguments=['-d', rviz_config],
            condition=IfCondition(start_rviz),
        ),
    ])
