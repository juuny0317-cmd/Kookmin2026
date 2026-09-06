from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    require_green_signal = LaunchConfiguration('require_green_signal')
    rotation_deg = LaunchConfiguration('rotation_deg')
    min_speed = LaunchConfiguration('min_speed')
    max_speed = LaunchConfiguration('max_speed')
    use_rviz_markers = LaunchConfiguration('use_rviz_markers')
    min_range_m = LaunchConfiguration('min_range_m')
    max_range_m = LaunchConfiguration('max_range_m')
    x_max_m = LaunchConfiguration('x_max_m')
    y_abs_max_m = LaunchConfiguration('y_abs_max_m')
    cluster_euclidean_gap_m = LaunchConfiguration('cluster_euclidean_gap_m')
    gap_min_width_m = LaunchConfiguration('gap_min_width_m')
    gap_max_width_m = LaunchConfiguration('gap_max_width_m')
    gap_expected_width_m = LaunchConfiguration('gap_expected_width_m')
    cone_spacing_m = LaunchConfiguration('cone_spacing_m')
    one_side_half_width_m = LaunchConfiguration('one_side_half_width_m')
    pair_min_normal_alignment = LaunchConfiguration(
        'pair_min_normal_alignment')
    pair_max_longitudinal_m = LaunchConfiguration(
        'pair_max_longitudinal_m')
    midpoint_reference_gate_m = LaunchConfiguration(
        'midpoint_reference_gate_m')
    control_x_offset_m = LaunchConfiguration('control_x_offset_m')
    min_lookahead_m = LaunchConfiguration('min_lookahead_m')
    max_lookahead_m = LaunchConfiguration('max_lookahead_m')
    lookahead_scale = LaunchConfiguration('lookahead_scale')
    command_publish_rate_hz = LaunchConfiguration('command_publish_rate_hz')

    return LaunchDescription([
        DeclareLaunchArgument('require_green_signal', default_value='true'),
        DeclareLaunchArgument('rotation_deg', default_value='0.0'),
        DeclareLaunchArgument('min_speed', default_value='2.0'),
        DeclareLaunchArgument('max_speed', default_value='4.0'),
        DeclareLaunchArgument('use_rviz_markers', default_value='true'),
        DeclareLaunchArgument('min_range_m', default_value='0.23'),
        DeclareLaunchArgument('max_range_m', default_value='2.50'),
        DeclareLaunchArgument('x_max_m', default_value='2.50'),
        DeclareLaunchArgument('y_abs_max_m', default_value='1.20'),
        DeclareLaunchArgument('cluster_euclidean_gap_m', default_value='0.12'),
        DeclareLaunchArgument('gap_min_width_m', default_value='0.50'),
        DeclareLaunchArgument('gap_max_width_m', default_value='1.00'),
        DeclareLaunchArgument('gap_expected_width_m', default_value='0.80'),
        DeclareLaunchArgument('cone_spacing_m', default_value='0.30'),
        DeclareLaunchArgument('one_side_half_width_m', default_value='0.40'),
        DeclareLaunchArgument(
            'pair_min_normal_alignment', default_value='0.45'),
        DeclareLaunchArgument(
            'pair_max_longitudinal_m', default_value='0.45'),
        DeclareLaunchArgument(
            'midpoint_reference_gate_m', default_value='0.45'),
        DeclareLaunchArgument('control_x_offset_m', default_value='0.30'),
        DeclareLaunchArgument('min_lookahead_m', default_value='0.30'),
        DeclareLaunchArgument('max_lookahead_m', default_value='0.60'),
        DeclareLaunchArgument('lookahead_scale', default_value='0.30'),
        DeclareLaunchArgument('command_publish_rate_hz', default_value='10.0'),
        Node(
            package='mission_cone_drive',
            executable='scan_rotator',
            name='scan_rotator_node',
            output='screen',
            parameters=[{
                'rotation_deg': rotation_deg,
            }],
        ),
        Node(
            package='mission_cone_drive',
            executable='preprocessing_node',
            name='preprocessing_node',
            output='screen',
            parameters=[{
                'require_green_signal': require_green_signal,
                'min_range_m': min_range_m,
                'max_range_m': max_range_m,
                'x_max_m': x_max_m,
                'y_abs_max_m': y_abs_max_m,
                'cluster_euclidean_gap_m': cluster_euclidean_gap_m,
            }],
        ),
        Node(
            package='mission_cone_drive',
            executable='path_planning_node',
            name='path_planning_node',
            output='screen',
            parameters=[{
                'require_green_signal': require_green_signal,
                'gap_min_width_m': gap_min_width_m,
                'gap_max_width_m': gap_max_width_m,
                'gap_expected_width_m': gap_expected_width_m,
                'cone_spacing_m': cone_spacing_m,
                'y_abs_max_m': y_abs_max_m,
                'one_side_half_width_m': one_side_half_width_m,
                'pair_min_normal_alignment': pair_min_normal_alignment,
                'pair_max_longitudinal_m': pair_max_longitudinal_m,
                'midpoint_reference_gate_m': midpoint_reference_gate_m,
            }],
        ),
        Node(
            package='mission_cone_drive',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            output='screen',
            parameters=[{
                'require_green_signal': require_green_signal,
                'min_speed': min_speed,
                'max_speed': max_speed,
                'control_x_offset_m': control_x_offset_m,
                'min_lookahead_m': min_lookahead_m,
                'max_lookahead_m': max_lookahead_m,
                'lookahead_scale': lookahead_scale,
                'command_publish_rate_hz': command_publish_rate_hz,
            }],
        ),
        Node(
            package='mission_cone_drive',
            executable='rviz_visualizer_node',
            name='rviz_visualizer_node',
            output='screen',
            condition=IfCondition(use_rviz_markers),
        ),
    ])
