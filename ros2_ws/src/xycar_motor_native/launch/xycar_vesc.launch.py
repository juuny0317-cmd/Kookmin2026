import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    enabled = LaunchConfiguration('enabled')
    port = LaunchConfiguration('port')
    auto_device_symlink = LaunchConfiguration('auto_device_symlink')
    auto_device_match = LaunchConfiguration('auto_device_match')
    handshake_timeout = LaunchConfiguration('handshake_timeout_s')
    reconnect_interval = LaunchConfiguration('reconnect_interval_s')
    telemetry_timeout = LaunchConfiguration('telemetry_timeout_s')
    motor_topic = LaunchConfiguration('motor_topic')
    control_mode = LaunchConfiguration('control_mode')
    duty_cycle_min = LaunchConfiguration('duty_cycle_min')
    duty_cycle_max = LaunchConfiguration('duty_cycle_max')
    duty_cycle_per_speed_unit = LaunchConfiguration(
        'duty_cycle_per_speed_unit')
    startup_duty_cycle = LaunchConfiguration('startup_duty_cycle')
    startup_duration_s = LaunchConfiguration('startup_duration_s')
    startup_exit_erpm = LaunchConfiguration('startup_exit_erpm')
    startup_exit_target_ratio = LaunchConfiguration(
        'startup_exit_target_ratio')
    startup_exit_confirm = LaunchConfiguration('startup_exit_confirm_s')
    erpm_per_speed_unit = LaunchConfiguration('erpm_per_speed_unit')
    erpm_kp = LaunchConfiguration('erpm_kp_duty_per_erpm')
    erpm_ki = LaunchConfiguration('erpm_ki_duty_per_erpm_s')
    erpm_integral_limit = LaunchConfiguration('erpm_integral_limit_duty')
    erpm_startup_integral = LaunchConfiguration(
        'erpm_startup_integral_duty')
    erpm_max_duty = LaunchConfiguration('erpm_max_duty')
    erpm_filter_alpha = LaunchConfiguration('erpm_filter_alpha')
    erpm_telemetry_timeout = LaunchConfiguration(
        'erpm_telemetry_timeout_s')
    duty_slew_rate = LaunchConfiguration('duty_slew_rate_per_s')
    watchdog_timeout = LaunchConfiguration('watchdog_timeout_s')
    config_file = os.path.join(
        get_package_share_directory('xycar_motor_native'),
        'config',
        'xycar_vesc.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'enabled',
            default_value='false',
            description=(
                'Safety interlock. Must be explicitly true to open the VESC '
                'and publish motor commands.'
            ),
        ),
        DeclareLaunchArgument(
            'port',
            default_value='auto',
            description=(
                'VESC serial device. "auto" tries /dev/ttyMOTOR, then a '
                'unique matching /dev/serial/by-id device, then an '
                'unambiguous /dev/ttyUSB device.'
            ),
        ),
        DeclareLaunchArgument(
            'auto_device_symlink', default_value='/dev/ttyMOTOR'),
        DeclareLaunchArgument(
            'auto_device_match', default_value='1a86_USB_Serial'),
        DeclareLaunchArgument(
            'handshake_timeout_s', default_value='2.0'),
        DeclareLaunchArgument(
            'reconnect_interval_s', default_value='1.0'),
        DeclareLaunchArgument(
            'telemetry_timeout_s', default_value='1.0'),
        DeclareLaunchArgument('motor_topic', default_value='/xycar_motor'),
        DeclareLaunchArgument(
            'control_mode',
            default_value='duty_speed',
            description='Motor output mode: duty_speed, duty_cycle, or speed.',
        ),
        DeclareLaunchArgument(
            'duty_cycle_min',
            default_value='-0.95',
            description='Hard lower duty limit applied by the VESC driver.',
        ),
        DeclareLaunchArgument(
            'duty_cycle_max',
            default_value='0.95',
            description='Hard upper duty limit applied by the VESC driver.',
        ),
        DeclareLaunchArgument(
            'duty_cycle_per_speed_unit',
            default_value='0.006',
            description='Duty-cycle generated for one Xycar speed unit.',
        ),
        DeclareLaunchArgument(
            'startup_duty_cycle',
            default_value='0.080',
            description='Fixed duty during the initial breakaway period.',
        ),
        DeclareLaunchArgument(
            'startup_duration_s',
            default_value='1.20',
            description='Maximum duration of the initial breakaway duty.',
        ),
        DeclareLaunchArgument(
            'startup_exit_erpm', default_value='1600.0'),
        DeclareLaunchArgument(
            'startup_exit_target_ratio', default_value='0.95'),
        DeclareLaunchArgument(
            'startup_exit_confirm_s', default_value='0.05'),
        DeclareLaunchArgument(
            'erpm_per_speed_unit', default_value='369.12'),
        DeclareLaunchArgument(
            'erpm_kp_duty_per_erpm', default_value='0.000010'),
        DeclareLaunchArgument(
            'erpm_ki_duty_per_erpm_s', default_value='0.000010'),
        DeclareLaunchArgument(
            'erpm_integral_limit_duty', default_value='0.10'),
        DeclareLaunchArgument(
            'erpm_startup_integral_duty', default_value='0.026'),
        DeclareLaunchArgument(
            'erpm_max_duty', default_value='0.25'),
        DeclareLaunchArgument(
            'erpm_filter_alpha', default_value='0.25'),
        DeclareLaunchArgument(
            'erpm_telemetry_timeout_s', default_value='0.25'),
        DeclareLaunchArgument(
            'duty_slew_rate_per_s', default_value='0.30'),
        DeclareLaunchArgument(
            'watchdog_timeout_s',
            default_value='0.30',
            description=(
                'Seconds without a fresh motor command before zero duty and '
                'center steering are commanded.'
            ),
        ),
        LogInfo(
            condition=UnlessCondition(enabled),
            msg='VESC motor driver is disabled by the safety interlock.',
        ),
        Node(
            package='vesc_driver',
            executable='vesc_driver_node',
            name='vesc_driver_node',
            output='screen',
            condition=IfCondition(enabled),
            parameters=[config_file, {
                'port': port,
                'auto_device_symlink': auto_device_symlink,
                'auto_device_match': auto_device_match,
                'handshake_timeout_s': ParameterValue(
                    handshake_timeout, value_type=float),
                'reconnect_interval_s': ParameterValue(
                    reconnect_interval, value_type=float),
                'telemetry_timeout_s': ParameterValue(
                    telemetry_timeout, value_type=float),
                'duty_cycle_min': ParameterValue(
                    duty_cycle_min, value_type=float),
                'duty_cycle_max': ParameterValue(
                    duty_cycle_max, value_type=float),
            }],
        ),
        Node(
            package='vesc_ackermann',
            executable='ackermann_to_vesc_node',
            name='ackermann_to_vesc_node',
            output='screen',
            condition=IfCondition(PythonExpression([
                "'", enabled, "' == 'true' and '",
                control_mode, "' == 'speed'",
            ])),
            parameters=[config_file],
            remappings=[('ackermann_cmd', '/ackermann_cmd')],
        ),
        Node(
            package='xycar_motor_native',
            executable='motor_command_adapter',
            name='xycar_motor_adapter',
            output='screen',
            condition=IfCondition(enabled),
            parameters=[config_file, {
                'input_topic': motor_topic,
                'control_mode': control_mode,
                'duty_cycle_per_speed_unit': ParameterValue(
                    duty_cycle_per_speed_unit, value_type=float),
                'startup_duty_cycle': ParameterValue(
                    startup_duty_cycle, value_type=float),
                'startup_duration_s': ParameterValue(
                    startup_duration_s, value_type=float),
                'startup_exit_erpm': ParameterValue(
                    startup_exit_erpm, value_type=float),
                'startup_exit_target_ratio': ParameterValue(
                    startup_exit_target_ratio, value_type=float),
                'startup_exit_confirm_s': ParameterValue(
                    startup_exit_confirm, value_type=float),
                'erpm_per_speed_unit': ParameterValue(
                    erpm_per_speed_unit, value_type=float),
                'erpm_kp_duty_per_erpm': ParameterValue(
                    erpm_kp, value_type=float),
                'erpm_ki_duty_per_erpm_s': ParameterValue(
                    erpm_ki, value_type=float),
                'erpm_integral_limit_duty': ParameterValue(
                    erpm_integral_limit, value_type=float),
                'erpm_startup_integral_duty': ParameterValue(
                    erpm_startup_integral, value_type=float),
                'erpm_max_duty': ParameterValue(
                    erpm_max_duty, value_type=float),
                'erpm_filter_alpha': ParameterValue(
                    erpm_filter_alpha, value_type=float),
                'erpm_telemetry_timeout_s': ParameterValue(
                    erpm_telemetry_timeout, value_type=float),
                'duty_slew_rate_per_s': ParameterValue(
                    duty_slew_rate, value_type=float),
                'watchdog_timeout_s': ParameterValue(
                    watchdog_timeout, value_type=float),
            }],
        ),
    ])
