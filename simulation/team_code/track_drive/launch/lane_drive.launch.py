from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def float_param(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def int_param(name):
    return ParameterValue(LaunchConfiguration(name), value_type=int)


def bool_param(name):
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/image_raw'),
        DeclareLaunchArgument('motor_topic', default_value='xycar_motor'),
        DeclareLaunchArgument('lane_mode', default_value='center'),
        DeclareLaunchArgument('debug_view', default_value='false'),

        DeclareLaunchArgument('straight_speed', default_value='8.0'),
        DeclareLaunchArgument('curve_speed', default_value='5.5'),
        DeclareLaunchArgument('min_speed', default_value='0.0'),
        DeclareLaunchArgument('max_angle', default_value='50.0'),
        DeclareLaunchArgument('max_angle_delta', default_value='4.0'),
        DeclareLaunchArgument('corner_max_angle_delta', default_value='5.5'),
        DeclareLaunchArgument('s_curve_speed', default_value='4.0'),
        DeclareLaunchArgument('s_curve_max_angle_delta', default_value='8.0'),
        DeclareLaunchArgument('s_curve_smooth_window', default_value='1'),

        DeclareLaunchArgument('sliding_margin_px', default_value='50'),
        DeclareLaunchArgument('sliding_minpix', default_value='30'),
        DeclareLaunchArgument('min_center_line_points', default_value='4'),
        DeclareLaunchArgument('path_fit_min_points', default_value='5'),
        DeclareLaunchArgument('lane_half_width_px', default_value='115.0'),
        DeclareLaunchArgument('path_smoothing_alpha', default_value='0.78'),

        DeclareLaunchArgument('pid_kp', default_value='0.28'),
        DeclareLaunchArgument('pid_kd', default_value='0.08'),
        DeclareLaunchArgument('heading_kp', default_value='14.0'),
        DeclareLaunchArgument('preview_heading_kp', default_value='6.0'),
        DeclareLaunchArgument('far_heading_kp', default_value='8.0'),
        DeclareLaunchArgument('s_curve_near_y_weight', default_value='0.58'),
        DeclareLaunchArgument('s_curve_preview_y_weight', default_value='0.37'),
        DeclareLaunchArgument('s_curve_far_y_weight', default_value='0.05'),
        DeclareLaunchArgument('s_curve_steer_boost', default_value='1.05'),
        DeclareLaunchArgument('target_forward_px', default_value='120.0'),
        DeclareLaunchArgument('preview_forward_px', default_value='190.0'),
        DeclareLaunchArgument('far_preview_forward_px', default_value='240.0'),

        DeclareLaunchArgument('yellow_h_low', default_value='22'),
        DeclareLaunchArgument('yellow_h_high', default_value='35'),
        DeclareLaunchArgument('yellow_s_low', default_value='130'),
        DeclareLaunchArgument('yellow_v_low', default_value='130'),
        DeclareLaunchArgument('white_s_high', default_value='30'),
        DeclareLaunchArgument('white_v_low', default_value='220'),
        DeclareLaunchArgument('histogram_threshold_ratio', default_value='0.38'),
        DeclareLaunchArgument('mask_min_component_area_px', default_value='35'),

        DeclareLaunchArgument('warp_top_left_x', default_value='130.0'),
        DeclareLaunchArgument('warp_top_right_x', default_value='500.0'),
        DeclareLaunchArgument('warp_top_y', default_value='280.0'),
        DeclareLaunchArgument('warp_bottom_left_x', default_value='0.0'),
        DeclareLaunchArgument('warp_bottom_right_x', default_value='640.0'),
        DeclareLaunchArgument('warp_bottom_y', default_value='385.0'),
        DeclareLaunchArgument('warp_dst_left_ratio', default_value='0.10'),
        DeclareLaunchArgument('warp_dst_right_ratio', default_value='0.90'),

        # Kept so older commands with these args still parse.
        DeclareLaunchArgument('pp_steer_gain', default_value='2.0'),
        DeclareLaunchArgument('pp_lookahead_base_px', default_value='70.0'),
        DeclareLaunchArgument('pp_lookahead_speed_gain', default_value='3.5'),

        Node(
            package='track_drive',
            executable='lane_drive',
            name='lane_drive',
            parameters=[{
                'image_topic': LaunchConfiguration('image_topic'),
                'motor_topic': LaunchConfiguration('motor_topic'),
                'lane_mode': LaunchConfiguration('lane_mode'),
                'debug_view': bool_param('debug_view'),

                'straight_speed': float_param('straight_speed'),
                'curve_speed': float_param('curve_speed'),
                'min_speed': float_param('min_speed'),
                'max_angle': float_param('max_angle'),
                'max_angle_delta': float_param('max_angle_delta'),
                'corner_max_angle_delta': float_param('corner_max_angle_delta'),
                's_curve_speed': float_param('s_curve_speed'),
                's_curve_max_angle_delta': float_param('s_curve_max_angle_delta'),
                's_curve_smooth_window': int_param('s_curve_smooth_window'),

                'sliding_margin_px': int_param('sliding_margin_px'),
                'sliding_minpix': int_param('sliding_minpix'),
                'min_center_line_points': int_param('min_center_line_points'),
                'path_fit_min_points': int_param('path_fit_min_points'),
                'lane_half_width_px': float_param('lane_half_width_px'),
                'path_smoothing_alpha': float_param('path_smoothing_alpha'),

                'pid_kp': float_param('pid_kp'),
                'pid_kd': float_param('pid_kd'),
                'heading_kp': float_param('heading_kp'),
                'preview_heading_kp': float_param('preview_heading_kp'),
                'far_heading_kp': float_param('far_heading_kp'),
                's_curve_near_y_weight': float_param('s_curve_near_y_weight'),
                's_curve_preview_y_weight': float_param('s_curve_preview_y_weight'),
                's_curve_far_y_weight': float_param('s_curve_far_y_weight'),
                's_curve_steer_boost': float_param('s_curve_steer_boost'),
                'target_forward_px': float_param('target_forward_px'),
                'preview_forward_px': float_param('preview_forward_px'),
                'far_preview_forward_px': float_param('far_preview_forward_px'),

                'yellow_h_low': int_param('yellow_h_low'),
                'yellow_h_high': int_param('yellow_h_high'),
                'yellow_s_low': int_param('yellow_s_low'),
                'yellow_v_low': int_param('yellow_v_low'),
                'white_s_high': int_param('white_s_high'),
                'white_v_low': int_param('white_v_low'),
                'histogram_threshold_ratio': float_param('histogram_threshold_ratio'),
                'mask_min_component_area_px': int_param('mask_min_component_area_px'),

                'warp_top_left_x': float_param('warp_top_left_x'),
                'warp_top_right_x': float_param('warp_top_right_x'),
                'warp_top_y': float_param('warp_top_y'),
                'warp_bottom_left_x': float_param('warp_bottom_left_x'),
                'warp_bottom_right_x': float_param('warp_bottom_right_x'),
                'warp_bottom_y': float_param('warp_bottom_y'),
                'warp_dst_left_ratio': float_param('warp_dst_left_ratio'),
                'warp_dst_right_ratio': float_param('warp_dst_right_ratio'),

                'pp_steer_gain': float_param('pp_steer_gain'),
                'pp_lookahead_base_px': float_param('pp_lookahead_base_px'),
                'pp_lookahead_speed_gain': float_param('pp_lookahead_speed_gain'),
            }],
        ),
    ])
