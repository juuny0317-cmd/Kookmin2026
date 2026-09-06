import os
import importlib.util
import difflib
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


FORWARDED_LAUNCH_ARGUMENTS = []
NUMERIC_LAUNCH_ARGUMENT_TYPES = {}


def infer_numeric_argument_types(actions):
    """Infer the intended ROS numeric type from each argument's default.

    ROS launch otherwise infers ``foo:=12`` as an integer even when the
    receiving node declared a double.  Field tuning commonly uses that short
    notation, so retain each declared default's intended type and normalize
    overrides before any nodes are created.
    """
    inferred = {}
    aliases = {}
    for action in actions:
        if not isinstance(action, DeclareLaunchArgument):
            continue
        substitutions = action.default_value
        if len(substitutions) != 1:
            continue
        default = substitutions[0]
        if isinstance(default, LaunchConfiguration):
            variable_name = default.variable_name
            if len(variable_name) == 1 and hasattr(variable_name[0], 'text'):
                aliases[action.name] = variable_name[0].text
            continue
        if not hasattr(default, 'text'):
            continue
        text = default.text.strip()
        try:
            int(text)
        except ValueError:
            try:
                float(text)
            except ValueError:
                continue
            inferred[action.name] = float
        else:
            inferred[action.name] = int

    unresolved = dict(aliases)
    while unresolved:
        progress = False
        for name, target in list(unresolved.items()):
            if target in inferred:
                inferred[name] = inferred[target]
                del unresolved[name]
                progress = True
        if not progress:
            break
    return inferred


def _requested_backend():
    for argument in sys.argv:
        if argument.startswith('execution_backend:='):
            return argument.split(':=', 1)[1].strip().lower()
    return 'auto'


def _needs_container_proxy():
    if os.environ.get('XYCAR_IN_CONTAINER') == '1':
        return False
    requested = _requested_backend()
    if requested == 'host':
        return False
    if requested == 'container':
        return True
    return (
        importlib.util.find_spec('torch') is None
        or importlib.util.find_spec('ultralytics') is None
    )


def validate_cli_argument_names(context):
    """Reject misspelled launch overrides instead of silently ignoring them."""
    del context
    declared = set(FORWARDED_LAUNCH_ARGUMENTS)
    unknown = []
    for token in sys.argv:
        if ':=' not in token:
            continue
        name = token.split(':=', 1)[0].strip()
        if name and name not in declared:
            suggestion = difflib.get_close_matches(name, declared, n=1)
            detail = f'{name!r}'
            if suggestion:
                detail += f' (did you mean {suggestion[0]!r}?)'
            unknown.append(detail)
    if unknown:
        raise RuntimeError(
            '[CONFIG ERROR] unknown launch argument(s): '
            + ', '.join(unknown)
            + '. Run with --show-args to list supported names.')
    return []


def launch_gpu_container(context):
    image = context.perform_substitution(
        LaunchConfiguration('container_image'))
    runtime_dir = '/tmp/xycar_runtime'
    os.makedirs(runtime_dir, exist_ok=True)
    host_domain_id = os.environ.get('ROS_DOMAIN_ID', '0')
    try:
        container_domain_id = str((int(host_domain_id) + 1) % 233)
    except ValueError as exc:
        raise RuntimeError(
            f'ROS_DOMAIN_ID must be an integer, got {host_domain_id!r}') \
            from exc
    command = [
        'docker', 'run', '--rm', '--init', '--runtime=nvidia',
        '--network=host', '--ipc=host',
        '--ulimit', 'memlock=-1', '--ulimit', 'stack=67108864',
        '--device-cgroup-rule', 'c 188:* rmw',
        '--device-cgroup-rule', 'c 166:* rmw',
        '--device-cgroup-rule', 'c 81:* rmw',
        '-e', 'XYCAR_IN_CONTAINER=1',
        '-e', f'ROS_DOMAIN_ID={container_domain_id}',
        '-e', 'ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET',
        '-e', 'RMW_IMPLEMENTATION=rmw_fastrtps_cpp',
        '-e', 'PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True',
        '-v', '/dev:/dev',
        '-v', f'{runtime_dir}:{runtime_dir}',
        '--label', 'com.xycar.role=integrated_drive',
    ]
    display = os.environ.get('DISPLAY', '')
    xauthority = os.environ.get(
        'XAUTHORITY', os.path.expanduser('~/.Xauthority'))
    if display and os.path.isdir('/tmp/.X11-unix'):
        command.extend([
            '-e', f'DISPLAY={display}',
            '-e', 'QT_X11_NO_MITSHM=1',
            '-v', '/tmp/.X11-unix:/tmp/.X11-unix:rw',
        ])
        if os.path.isfile(xauthority) and os.access(xauthority, os.R_OK):
            command.extend([
                '-e', 'XAUTHORITY=/tmp/xycar.Xauthority',
                '-v', f'{os.path.realpath(xauthority)}:'
                      '/tmp/xycar.Xauthority:ro',
            ])
    proxy_overrides = {
        'execution_backend',
        'container_image',
        'mission_start_service_name',
        'mission_stop_service_name',
        'mission_pause_service_name',
        'mission_resume_service_name',
        'mission_toggle_pause_service_name',
        'enable_spacebar_pause',
    }
    forwarded = []
    for name in FORWARDED_LAUNCH_ARGUMENTS:
        if name in proxy_overrides:
            continue
        resolved = context.perform_substitution(LaunchConfiguration(name))
        # ROS 2 CLI rejects ``name:=``. Empty values already match the inner
        # launch defaults, so omitting them preserves the same configuration.
        if resolved == '':
            continue
        forwarded.append(f'{name}:={resolved}')
    command.extend([
        image,
        'ros2', 'launch', 'mission_cone_drive',
        'integrated_drive.launch.py',
        'execution_backend:=host',
        *forwarded,
        'mission_start_service_name:=/_start_integrated_drive_internal',
        'mission_stop_service_name:=/_stop_integrated_drive_internal',
        'mission_pause_service_name:=/_pause_integrated_drive_internal',
        'mission_resume_service_name:=/_resume_integrated_drive_internal',
        'mission_toggle_pause_service_name:='
        '/_toggle_pause_integrated_drive_internal',
        'enable_spacebar_pause:=false',
    ])
    return [
        LogInfo(msg=(
            '[RUNTIME] Host CUDA Python packages are unavailable; '
            f'starting the integrated stack in {image}. Public drive-control '
            'services are provided by the host service proxy. '
            f'ROS domains: host={host_domain_id}, '
            f'container={container_domain_id}.')),
        ExecuteProcess(
            cmd=command,
            output='screen',
            emulate_tty=True,
            sigterm_timeout='10',
            sigkill_timeout='3',
        ),
    ]


def include_package_launch(
        package_name, launch_filename, condition, launch_arguments=None):
    launch_path = PathJoinSubstitution([
        FindPackageShare(package_name),
        'launch',
        launch_filename,
    ])
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        condition=IfCondition(condition),
        launch_arguments=(launch_arguments or {}).items(),
    )


def validate_and_log_configuration(context):
    validate_cli_argument_names(context)

    def value(name):
        return context.perform_substitution(LaunchConfiguration(name))

    def number(name):
        try:
            return float(value(name))
        except ValueError as error:
            raise RuntimeError(
                f'[CONFIG ERROR] {name} must be numeric, got {value(name)!r}'
            ) from error

    def enabled(name):
        return value(name).strip().lower() in ('1', 'true', 'yes', 'on')

    errors = []
    for name, expected_type in NUMERIC_LAUNCH_ARGUMENT_TYPES.items():
        raw_value = value(name)
        try:
            if expected_type is float:
                normalized = repr(float(raw_value))
            else:
                numeric = float(raw_value)
                if not numeric.is_integer():
                    raise ValueError('not an integer')
                normalized = str(int(numeric))
        except ValueError:
            expected_name = (
                'floating-point' if expected_type is float else 'integer')
            errors.append(
                f'{name} must be {expected_name}, got {raw_value!r}')
            continue
        context.launch_configurations[name] = normalized

    cone_min = number('cone_min_speed')
    cone_max = number('cone_max_speed')
    cone_entry = number('cone_entry_speed')
    if cone_min < 0.0 or cone_max < 0.0 or cone_min > cone_max:
        errors.append('cone speeds require 0 <= min_speed <= max_speed')
    if not cone_min <= cone_entry <= cone_max:
        errors.append(
            'cone_entry_speed must be between cone_min_speed and '
            'cone_max_speed')
    if number('lidar_dbscan_eps_m') <= 0.0:
        errors.append('lidar_dbscan_eps_m must be > 0')
    if number('lidar_dbscan_min_samples') < 1.0:
        errors.append('lidar_dbscan_min_samples must be >= 1')
    if number('cone_entry_fusion_max_range_m') <= 0.0:
        errors.append('cone_entry_fusion_max_range_m must be > 0')
    if (
        number('cone_prearm_max_range_m')
        < number('cone_entry_fusion_max_range_m')
    ):
        errors.append(
            'cone_prearm_max_range_m must be >= '
            'cone_entry_fusion_max_range_m')
    if number('cone_prearm_dbscan_eps_m') <= 0.0:
        errors.append('cone_prearm_dbscan_eps_m must be > 0')
    if number('cone_prearm_dbscan_min_samples') < 1.0:
        errors.append('cone_prearm_dbscan_min_samples must be >= 1')
    if number('cone_prearm_window') < 1.0:
        errors.append('cone_prearm_window must be >= 1')
    if number('cone_bbox_prearm_min_detections') < 1.0:
        errors.append('cone_bbox_prearm_min_detections must be >= 1')
    if number('cone_bbox_prearm_clear_frames') < 1.0:
        errors.append('cone_bbox_prearm_clear_frames must be >= 1')
    if not 1.0 <= number('cone_prearm_min_matches') <= number(
            'cone_prearm_window'):
        errors.append(
            'cone_prearm_min_matches must be between 1 and '
            'cone_prearm_window')
    for nonnegative_name in (
            'cone_prearm_min_bbox_area_px',
            'cone_prearm_min_bottom_y_px',
            'cone_prearm_hold_s',
            'cone_prearm_speed_cap'):
        if number(nonnegative_name) < 0.0:
            errors.append(f'{nonnegative_name} must be >= 0')
    steering_straight_rate = number(
        'stanley_steering_rate_straight_per_s')
    steering_curve_rate = number('stanley_steering_rate_curve_per_s')
    steering_reversal_rate = number(
        'stanley_steering_rate_reversal_per_s')
    if not 0.0 <= steering_straight_rate <= steering_curve_rate:
        errors.append(
            'Stanley steering rates require 0 <= straight <= curve')
    if steering_reversal_rate < steering_curve_rate:
        errors.append(
            'stanley_steering_rate_reversal_per_s must be >= curve rate')
    steering_risk_low = number('stanley_steering_rate_risk_low')
    steering_risk_high = number('stanley_steering_rate_risk_high')
    if not 0.0 <= steering_risk_low < steering_risk_high <= 1.0:
        errors.append(
            'Stanley steering risks require 0 <= low < high <= 1')
    if number('stanley_steering_rate_hold_frames') < 0.0:
        errors.append('stanley_steering_rate_hold_frames must be >= 0')
    for rate_name in (
            'lane_max_rate_hz', 'lane_yolo_max_rate_hz',
            'scene_max_rate_hz', 'scene_yolo_max_rate_hz'):
        if number(rate_name) < 0.0:
            errors.append(f'{rate_name} must be >= 0')
    if number('shortcut_exit_min_turn_duration_s') < 0.0:
        errors.append('shortcut_exit_min_turn_duration_s must be >= 0')
    if not 1.0 <= number('shortcut_left_signal_min_matches') <= number(
            'shortcut_left_signal_window'):
        errors.append(
            'shortcut_left_signal_min_matches must be between 1 and '
            'shortcut_left_signal_window')
    if number('lane_curve_center_agreement_threshold_px') < 0.0:
        errors.append(
            'lane_curve_center_agreement_threshold_px must be >= 0')
    if not 0.0 <= number(
            'lane_curve_center_correction_weight') <= 1.0:
        errors.append(
            'lane_curve_center_correction_weight must be between 0 and 1')
    if number('lane_curve_center_hold_frames') < 0.0:
        errors.append('lane_curve_center_hold_frames must be >= 0')
    if number('lane_straight_center_agreement_threshold_px') < 0.0:
        errors.append(
            'lane_straight_center_agreement_threshold_px must be >= 0')
    if not 0.0 <= number(
            'lane_straight_center_correction_weight') <= 1.0:
        errors.append(
            'lane_straight_center_correction_weight must be between 0 and 1')
    if number('lane_straight_center_max_jump_px') < 0.0:
        errors.append('lane_straight_center_max_jump_px must be >= 0')
    if number('lane_straight_center_jump_confirm_frames') < 1.0:
        errors.append(
            'lane_straight_center_jump_confirm_frames must be >= 1')
    if number('lane_obstacle_recovery_duration_s') < 0.0:
        errors.append('lane_obstacle_recovery_duration_s must be >= 0')

    strong_speed = number('lane_curve_strong_speed')
    medium_speed = number('lane_curve_medium_speed')
    mild_speed = number('lane_curve_mild_speed')
    recovery_speed = number('lane_straight_recovery_speed')
    straight_speed = number('lane_straight_speed')
    if not (
        0.0 <= strong_speed <= medium_speed
        <= mild_speed <= straight_speed
    ):
        errors.append(
            'lane speeds require 0 <= strong <= medium <= mild '
            '<= straight')
    if not 0.0 <= recovery_speed <= straight_speed:
        errors.append(
            'lane recovery speed requires 0 <= recovery <= straight')
    medium_threshold = number('lane_curve_medium_threshold')
    strong_threshold = number('lane_curve_strong_threshold')
    if not 0.0 <= medium_threshold < strong_threshold <= 1.0:
        errors.append(
            'lane curve thresholds require '
            '0 <= medium < strong <= 1')
    for rate_name in (
            'lane_curve_accel_rate', 'lane_straight_accel_rate',
            'lane_curve_decel_rate', 'lane_straight_decel_rate',
            'cone_speed_accel_rate_per_s',
            'cone_speed_decel_rate_per_s'):
        if number(rate_name) < 0.0:
            errors.append(f'{rate_name} must be >= 0')

    for kind in ('dynamic', 'static'):
        obstacle_speed = number(f'{kind}_obstacle_speed')
        speed_distance = number(f'{kind}_speed_trigger_distance_m')
        avoid_distance = number(f'{kind}_overtake_trigger_distance_m')
        lane_distance = number(f'{kind}_lane_determination_distance_m')
        if obstacle_speed < 0.0:
            errors.append(f'{kind}_obstacle_speed must be >= 0')
        if not 0.0 < avoid_distance <= speed_distance:
            errors.append(
                f'{kind} distances require 0 < overtake <= speed_trigger')
        if lane_distance < avoid_distance:
            errors.append(
                f'{kind}_lane_determination_distance_m must be >= '
                f'{kind}_overtake_trigger_distance_m')
        if number(f'{kind}_event_duration_s') < 0.0:
            errors.append(f'{kind}_event_duration_s must be >= 0')
        for decision in ('speed', 'avoid'):
            matches_name = f'{kind}_{decision}_confirm_min_matches'
            window_name = f'{kind}_{decision}_confirm_window_frames'
            if not 1.0 <= number(matches_name) <= number(window_name):
                errors.append(
                    f'{matches_name} must be between 1 and '
                    f'{window_name}')
        if number(f'{kind}_rearm_clear_frames') < 1.0:
            errors.append(f'{kind}_rearm_clear_frames must be >= 1')

    if enabled('start_motor_driver') and enabled('start_ultrasonic'):
        motor_port = value('motor_port')
        motor_candidates = []
        if motor_port != 'auto':
            motor_candidates.append(motor_port)
        else:
            motor_candidates.extend([
                value('motor_auto_device_symlink'),
                '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
            ])
        ultrasonic_path = '/dev/ttySonic'
        if os.path.exists(ultrasonic_path):
            ultrasonic_real = os.path.realpath(ultrasonic_path)
            if any(
                    os.path.exists(path)
                    and os.path.realpath(path) == ultrasonic_real
                    for path in motor_candidates):
                errors.append(
                    'motor and ultrasonic resolve to the same serial device; '
                    'disable start_ultrasonic or fix the udev aliases')

    if errors:
        raise RuntimeError('[CONFIG ERROR] ' + '; '.join(errors))

    return [
        LogInfo(msg=(
            '[DRIVE CONFIG] '
            f'straight={value("lane_straight_speed")}, '
            'curve(strong/medium/mild)='
            f'{value("lane_curve_strong_speed")}/'
            f'{value("lane_curve_medium_speed")}/'
            f'{value("lane_curve_mild_speed")}, '
            f'recovery={value("lane_straight_recovery_speed")}, '
            'decel(straight/curve)='
            f'{value("lane_straight_decel_rate")}/'
            f'{value("lane_curve_decel_rate")}')),
        LogInfo(msg=(
            '[OBSTACLE CONFIG] '
            'dynamic(speed/approach/avoid/hold)='
            f'{value("dynamic_obstacle_speed")}/'
            f'{value("dynamic_speed_trigger_distance_m")}/'
            f'{value("dynamic_overtake_trigger_distance_m")}/'
            f'{value("dynamic_event_duration_s")}, '
            'confirm(speed/avoid)='
            f'{value("dynamic_speed_confirm_min_matches")}-of-'
            f'{value("dynamic_speed_confirm_window_frames")}/'
            f'{value("dynamic_avoid_confirm_min_matches")}-of-'
            f'{value("dynamic_avoid_confirm_window_frames")}, '
            'static(speed/approach/avoid/hold)='
            f'{value("static_obstacle_speed")}/'
            f'{value("static_speed_trigger_distance_m")}/'
            f'{value("static_overtake_trigger_distance_m")}/'
            f'{value("static_event_duration_s")}, '
            'confirm(speed/avoid)='
            f'{value("static_speed_confirm_min_matches")}-of-'
            f'{value("static_speed_confirm_window_frames")}/'
            f'{value("static_avoid_confirm_min_matches")}-of-'
            f'{value("static_avoid_confirm_window_frames")}')),
        LogInfo(msg=(
            '[CONE CONFIG] '
            f'controller={value("cone_controller")}, '
            'drive_dbscan=0.15/5, diameter=0.30m, angle_bin=36deg, '
            'path=cubic_spline_100, lookahead=0.70-2.50m, '
            f'speed={value("cone_min_speed")}-'
            f'{value("cone_entry_speed")}-'
            f'{value("cone_max_speed")}, '
            'accel/decel='
            f'{value("cone_speed_accel_rate_per_s")}/'
            f'{value("cone_speed_decel_rate_per_s")}, steering=direct, '
            f'fusion_dbscan={value("lidar_dbscan_eps_m")}/'
            f'{value("lidar_dbscan_min_samples")}')),
        LogInfo(msg=(
            '[YOLO CONFIG] '
            f'lane_router={value("lane_max_rate_hz")}Hz, '
            f'lane_inference_cap={value("lane_yolo_max_rate_hz")}Hz, '
            f'scene_router={value("scene_max_rate_hz")}Hz, '
            f'scene_inference_cap={value("scene_yolo_max_rate_hz")}Hz, '
            f'lane_size={value("lane_inference_image_size")}, '
            f'scene_size={value("scene_inference_image_size")}')),
        LogInfo(msg=(
            '[VESC CONFIG] '
            f'enabled={value("start_motor_driver")}, '
            f'port={value("motor_port")}, '
            f'auto_match={value("motor_auto_device_match")}, '
            f'handshake_timeout={value("vesc_handshake_timeout_s")}s, '
            f'reconnect={value("vesc_reconnect_interval_s")}s')),
    ]


def validate_configuration_only(context):
    """Run host-side validation without duplicating inner runtime logs."""
    validate_and_log_configuration(context)
    return []


def generate_launch_description():
    start_camera = LaunchConfiguration('start_camera')
    start_lidar = LaunchConfiguration('start_lidar')
    start_ultrasonic = LaunchConfiguration('start_ultrasonic')
    start_yolo = LaunchConfiguration('start_yolo')
    start_lane_stack = LaunchConfiguration('start_lane_stack')
    start_rviz = LaunchConfiguration('start_rviz')
    start_active = LaunchConfiguration('start_active')
    start_motor_driver = LaunchConfiguration('start_motor_driver')
    mission_start_service_name = LaunchConfiguration(
        'mission_start_service_name')
    mission_stop_service_name = LaunchConfiguration(
        'mission_stop_service_name')
    mission_pause_service_name = LaunchConfiguration(
        'mission_pause_service_name')
    mission_resume_service_name = LaunchConfiguration(
        'mission_resume_service_name')
    mission_toggle_pause_service_name = LaunchConfiguration(
        'mission_toggle_pause_service_name')
    enable_spacebar_pause = LaunchConfiguration('enable_spacebar_pause')
    motor_port = LaunchConfiguration('motor_port')
    motor_auto_device_symlink = LaunchConfiguration(
        'motor_auto_device_symlink')
    motor_auto_device_match = LaunchConfiguration(
        'motor_auto_device_match')
    vesc_handshake_timeout_s = LaunchConfiguration(
        'vesc_handshake_timeout_s')
    vesc_reconnect_interval_s = LaunchConfiguration(
        'vesc_reconnect_interval_s')
    vesc_telemetry_timeout_s = LaunchConfiguration(
        'vesc_telemetry_timeout_s')
    use_sim_time = LaunchConfiguration('use_sim_time')
    motor_topic = LaunchConfiguration('motor_topic')
    show_yolo_view = LaunchConfiguration('show_yolo_view')
    show_lane_view = LaunchConfiguration('show_lane_view')
    show_overtake_view = LaunchConfiguration('show_overtake_view')
    model_filename = LaunchConfiguration('model_filename')
    cone_conf_threshold = LaunchConfiguration('cone_conf_threshold')
    yolo_image_qos_reliability = LaunchConfiguration(
        'yolo_image_qos_reliability')
    enable_lane_pipeline = LaunchConfiguration('enable_lane_pipeline')
    enable_scene_pipeline = LaunchConfiguration('enable_scene_pipeline')
    enable_drive_control = LaunchConfiguration('enable_drive_control')
    enable_shortcut_left_turn = LaunchConfiguration(
        'enable_shortcut_left_turn')
    shortcut_auto_trigger_after_s = LaunchConfiguration(
        'shortcut_auto_trigger_after_s')
    shortcut_auto_trigger_state = LaunchConfiguration(
        'shortcut_auto_trigger_state')
    shortcut_prepare_speed_limit = LaunchConfiguration(
        'shortcut_prepare_speed_limit')
    shortcut_turn_speed_limit = LaunchConfiguration(
        'shortcut_turn_speed_limit')
    shortcut_cruise_speed_limit = LaunchConfiguration(
        'shortcut_cruise_speed_limit')
    shortcut_steering_limit = LaunchConfiguration(
        'shortcut_steering_limit')
    shortcut_left_signal_window = LaunchConfiguration(
        'shortcut_left_signal_window')
    shortcut_left_signal_min_matches = LaunchConfiguration(
        'shortcut_left_signal_min_matches')
    shortcut_fallback_commit_s = LaunchConfiguration(
        'shortcut_fallback_commit_s')
    shortcut_entry_max_heading_deg = LaunchConfiguration(
        'shortcut_entry_max_heading_deg')
    shortcut_min_cruise_duration_s = LaunchConfiguration(
        'shortcut_min_cruise_duration_s')
    shortcut_crossline_window = LaunchConfiguration(
        'shortcut_crossline_window')
    shortcut_crossline_min_matches = LaunchConfiguration(
        'shortcut_crossline_min_matches')
    shortcut_exit_min_turn_duration_s = LaunchConfiguration(
        'shortcut_exit_min_turn_duration_s')
    shortcut_exit_alignment_window = LaunchConfiguration(
        'shortcut_exit_alignment_window')
    shortcut_exit_alignment_min_matches = LaunchConfiguration(
        'shortcut_exit_alignment_min_matches')
    shortcut_exit_handoff_duration_s = LaunchConfiguration(
        'shortcut_exit_handoff_duration_s')
    shortcut_exit_max_heading_deg = LaunchConfiguration(
        'shortcut_exit_max_heading_deg')
    input_image_topic = LaunchConfiguration('input_image_topic')
    camera_qos_reliability = LaunchConfiguration('camera_qos_reliability')
    publish_performance_stats = LaunchConfiguration(
        'publish_performance_stats')
    performance_log_period_s = LaunchConfiguration(
        'performance_log_period_s')
    publish_pipeline_timing = LaunchConfiguration(
        'publish_pipeline_timing')
    pipeline_timing_topic = LaunchConfiguration('pipeline_timing_topic')
    publish_resource_stats = LaunchConfiguration('publish_resource_stats')
    resource_stats_rate_hz = LaunchConfiguration('resource_stats_rate_hz')
    resource_stats_output_path = LaunchConfiguration(
        'resource_stats_output_path')
    resource_stats_process_pattern = LaunchConfiguration(
        'resource_stats_process_pattern')
    resource_stats_gpu_index = LaunchConfiguration(
        'resource_stats_gpu_index')
    resource_stats_profile_threads = LaunchConfiguration(
        'resource_stats_profile_threads')
    stanley_event_driven = LaunchConfiguration('stanley_event_driven')
    mission_event_driven_lane = LaunchConfiguration(
        'mission_event_driven_lane')
    publish_camera_timing = LaunchConfiguration('publish_camera_timing')
    camera_timing_topic = LaunchConfiguration('camera_timing_topic')
    future_stamp_tolerance_s = LaunchConfiguration(
        'future_stamp_tolerance_s')

    lane_model_filename = LaunchConfiguration('lane_model_filename')
    lane_image_topic = LaunchConfiguration('lane_image_topic')
    lane_image_width = LaunchConfiguration('lane_image_width')
    lane_image_height = LaunchConfiguration('lane_image_height')
    lane_max_rate_hz = LaunchConfiguration('lane_max_rate_hz')
    frame_router_event_driven = LaunchConfiguration(
        'frame_router_event_driven')
    lane_yolo_max_rate_hz = LaunchConfiguration('lane_yolo_max_rate_hz')
    lane_inference_image_size = LaunchConfiguration(
        'lane_inference_image_size')
    lane_class_names = LaunchConfiguration('lane_class_names')
    lane_torch_threads = LaunchConfiguration('lane_torch_threads')
    lane_torch_interop_threads = LaunchConfiguration(
        'lane_torch_interop_threads')
    lane_opencv_threads = LaunchConfiguration('lane_opencv_threads')
    lane_torch_compile_mode = LaunchConfiguration(
        'lane_torch_compile_mode')
    lane_compile_warmup_iterations = LaunchConfiguration(
        'lane_compile_warmup_iterations')
    lane_yolo_device = LaunchConfiguration('lane_yolo_device')
    lane_yolo_fp16 = LaunchConfiguration('lane_yolo_fp16')
    lane_yolo_runtime_report_path = LaunchConfiguration(
        'lane_yolo_runtime_report_path')
    lane_process_nice = LaunchConfiguration('lane_process_nice')
    lane_cpu_affinity = LaunchConfiguration('lane_cpu_affinity')
    lane_image_qos_reliability = LaunchConfiguration(
        'lane_image_qos_reliability')
    lane_annotated_max_rate_hz = LaunchConfiguration(
        'lane_annotated_max_rate_hz')
    show_lane_yolo_view = LaunchConfiguration('show_lane_yolo_view')
    show_lane_processing_view = LaunchConfiguration(
        'show_lane_processing_view')
    lane_base_input_width = LaunchConfiguration('lane_base_input_width')
    lane_base_input_height = LaunchConfiguration('lane_base_input_height')
    lane_center_height_threshold_px = LaunchConfiguration(
        'lane_center_height_threshold_px')
    lane_center_adaptive_block_size_px = LaunchConfiguration(
        'lane_center_adaptive_block_size_px')
    lane_center_hough_threshold = LaunchConfiguration(
        'lane_center_hough_threshold')
    lane_center_hough_min_line_length_px = LaunchConfiguration(
        'lane_center_hough_min_line_length_px')
    lane_center_hough_max_line_gap_px = LaunchConfiguration(
        'lane_center_hough_max_line_gap_px')
    lane_center_similarity_threshold_px = LaunchConfiguration(
        'lane_center_similarity_threshold_px')
    lane_center_rmse_threshold_px = LaunchConfiguration(
        'lane_center_rmse_threshold_px')
    lane_center_bottom_box_height_threshold_px = LaunchConfiguration(
        'lane_center_bottom_box_height_threshold_px')
    lane_center_bottom_point_height_diff_threshold_px = LaunchConfiguration(
        'lane_center_bottom_point_height_diff_threshold_px')
    lane_center_min_bbox_width_px = LaunchConfiguration(
        'lane_center_min_bbox_width_px')
    lane_center_min_bbox_height_px = LaunchConfiguration(
        'lane_center_min_bbox_height_px')
    lane_center_x_distance_threshold_px = LaunchConfiguration(
        'lane_center_x_distance_threshold_px')
    lane_center_point_merge_distance_px = LaunchConfiguration(
        'lane_center_point_merge_distance_px')
    lane_center_spline_smoothing_px2 = LaunchConfiguration(
        'lane_center_spline_smoothing_px2')
    lane_canny_blur_kernel_px = LaunchConfiguration(
        'lane_canny_blur_kernel_px')
    lane_object_grid_cell_size_px = LaunchConfiguration(
        'lane_object_grid_cell_size_px')
    lane_curve_heading_recovery_enabled = LaunchConfiguration(
        'lane_curve_heading_recovery_enabled')
    lane_curve_heading_disagreement_deg = LaunchConfiguration(
        'lane_curve_heading_disagreement_deg')
    lane_curve_heading_recovery_weight = LaunchConfiguration(
        'lane_curve_heading_recovery_weight')
    lane_curve_heading_confirm_frames = LaunchConfiguration(
        'lane_curve_heading_confirm_frames')
    lane_curve_heading_curve_max_fit_error_ratio = LaunchConfiguration(
        'lane_curve_heading_curve_max_fit_error_ratio')
    lane_curve_heading_extrapolation_margin_ratio = LaunchConfiguration(
        'lane_curve_heading_extrapolation_margin_ratio')
    lane_adaptive_heading_enabled = LaunchConfiguration(
        'lane_adaptive_heading_enabled')
    lane_heading_preview_min_ratio = LaunchConfiguration(
        'lane_heading_preview_min_ratio')
    lane_heading_preview_max_ratio = LaunchConfiguration(
        'lane_heading_preview_max_ratio')
    lane_heading_curvature_scale = LaunchConfiguration(
        'lane_heading_curvature_scale')
    lane_heading_curvature_deadzone = LaunchConfiguration(
        'lane_heading_curvature_deadzone')
    lane_heading_filter_alpha_straight = LaunchConfiguration(
        'lane_heading_filter_alpha_straight')
    lane_heading_filter_alpha_curve = LaunchConfiguration(
        'lane_heading_filter_alpha_curve')
    lane_heading_filter_alpha_fallback = LaunchConfiguration(
        'lane_heading_filter_alpha_fallback')
    lane_heading_curve_hold_frames = LaunchConfiguration(
        'lane_heading_curve_hold_frames')
    lane_heading_max_step_deg = LaunchConfiguration(
        'lane_heading_max_step_deg')
    lane_lateral_innovation_straight_px = LaunchConfiguration(
        'lane_lateral_innovation_straight_px')
    lane_lateral_innovation_curve_px = LaunchConfiguration(
        'lane_lateral_innovation_curve_px')
    lane_lateral_max_rate_straight_px_s = LaunchConfiguration(
        'lane_lateral_max_rate_straight_px_s')
    lane_lateral_lane_half_width_px = LaunchConfiguration(
        'lane_lateral_lane_half_width_px')
    lane_lateral_outer_pair_tolerance_px = LaunchConfiguration(
        'lane_lateral_outer_pair_tolerance_px')
    lane_lateral_outer_pair_hold_frames = LaunchConfiguration(
        'lane_lateral_outer_pair_hold_frames')
    lane_straight_center_agreement_threshold_px = LaunchConfiguration(
        'lane_straight_center_agreement_threshold_px')
    lane_straight_center_correction_weight = LaunchConfiguration(
        'lane_straight_center_correction_weight')
    lane_straight_center_max_jump_px = LaunchConfiguration(
        'lane_straight_center_max_jump_px')
    lane_straight_center_jump_confirm_frames = LaunchConfiguration(
        'lane_straight_center_jump_confirm_frames')
    lane_obstacle_recovery_duration_s = LaunchConfiguration(
        'lane_obstacle_recovery_duration_s')
    lane_curve_center_agreement_threshold_px = LaunchConfiguration(
        'lane_curve_center_agreement_threshold_px')
    lane_curve_center_correction_weight = LaunchConfiguration(
        'lane_curve_center_correction_weight')
    lane_curve_center_hold_frames = LaunchConfiguration(
        'lane_curve_center_hold_frames')

    scene_model_filename = LaunchConfiguration('scene_model_filename')
    scene_image_topic = LaunchConfiguration('scene_image_topic')
    scene_image_width = LaunchConfiguration('scene_image_width')
    scene_image_height = LaunchConfiguration('scene_image_height')
    scene_max_rate_hz = LaunchConfiguration('scene_max_rate_hz')
    scene_yolo_max_rate_hz = LaunchConfiguration(
        'scene_yolo_max_rate_hz')
    scene_inference_image_size = LaunchConfiguration(
        'scene_inference_image_size')
    scene_class_names = LaunchConfiguration('scene_class_names')
    scene_class_name_aliases = LaunchConfiguration(
        'scene_class_name_aliases')
    scene_general_conf_threshold = LaunchConfiguration(
        'scene_general_conf_threshold')
    scene_obstacle_conf_threshold = LaunchConfiguration(
        'scene_obstacle_conf_threshold')
    scene_enable_checkerboard = LaunchConfiguration(
        'scene_enable_checkerboard')
    scene_torch_threads = LaunchConfiguration('scene_torch_threads')
    scene_torch_interop_threads = LaunchConfiguration(
        'scene_torch_interop_threads')
    scene_opencv_threads = LaunchConfiguration('scene_opencv_threads')
    scene_torch_compile_mode = LaunchConfiguration(
        'scene_torch_compile_mode')
    scene_compile_warmup_iterations = LaunchConfiguration(
        'scene_compile_warmup_iterations')
    scene_yolo_device = LaunchConfiguration('scene_yolo_device')
    scene_yolo_fp16 = LaunchConfiguration('scene_yolo_fp16')
    scene_yolo_runtime_report_path = LaunchConfiguration(
        'scene_yolo_runtime_report_path')
    scene_process_nice = LaunchConfiguration('scene_process_nice')
    scene_cpu_affinity = LaunchConfiguration('scene_cpu_affinity')
    scene_image_qos_reliability = LaunchConfiguration(
        'scene_image_qos_reliability')
    scene_detection_max_age_s = LaunchConfiguration(
        'scene_detection_max_age_s')
    scene_annotated_max_rate_hz = LaunchConfiguration(
        'scene_annotated_max_rate_hz')
    show_scene_yolo_view = LaunchConfiguration('show_scene_yolo_view')
    show_traffic_light_view = LaunchConfiguration(
        'show_traffic_light_view')
    traffic_direct_state_min_confidence = LaunchConfiguration(
        'traffic_direct_state_min_confidence')
    traffic_direct_red_min_confidence = LaunchConfiguration(
        'traffic_direct_red_min_confidence')
    traffic_direct_green_min_confidence = LaunchConfiguration(
        'traffic_direct_green_min_confidence')
    traffic_direct_left_min_confidence = LaunchConfiguration(
        'traffic_direct_left_min_confidence')
    traffic_stable_frames = LaunchConfiguration('traffic_stable_frames')
    traffic_left_is_green = LaunchConfiguration('traffic_left_is_green')
    traffic_red_latch_until_green = LaunchConfiguration(
        'traffic_red_latch_until_green')
    traffic_red_latch_confirm_frames = LaunchConfiguration(
        'traffic_red_latch_confirm_frames')
    sync_slop = LaunchConfiguration('sync_slop')
    sync_queue_size = LaunchConfiguration('sync_queue_size')
    overtake_sensor_timeout_s = LaunchConfiguration(
        'overtake_sensor_timeout_s')
    road_curve_max_time_delta_s = LaunchConfiguration(
        'road_curve_max_time_delta_s')
    obstacle_lidar_min_range_m = LaunchConfiguration(
        'obstacle_lidar_min_range_m')
    obstacle_lidar_max_range_m = LaunchConfiguration(
        'obstacle_lidar_max_range_m')
    obstacle_lidar_cluster_min_points = LaunchConfiguration(
        'obstacle_lidar_cluster_min_points')
    obstacle_lidar_cluster_base_gap_m = LaunchConfiguration(
        'obstacle_lidar_cluster_base_gap_m')
    obstacle_lidar_cluster_range_gap_scale = LaunchConfiguration(
        'obstacle_lidar_cluster_range_gap_scale')
    obstacle_lidar_cluster_max_width_m = LaunchConfiguration(
        'obstacle_lidar_cluster_max_width_m')
    obstacle_lidar_distance_percentile = LaunchConfiguration(
        'obstacle_lidar_distance_percentile')
    obstacle_lidar_association_padding_px = LaunchConfiguration(
        'obstacle_lidar_association_padding_px')
    obstacle_lidar_association_max_above_px = LaunchConfiguration(
        'obstacle_lidar_association_max_above_px')
    obstacle_lidar_camera_max_delta_s = LaunchConfiguration(
        'obstacle_lidar_camera_max_delta_s')
    obstacle_lidar_tracking_enabled = LaunchConfiguration(
        'obstacle_lidar_tracking_enabled')
    obstacle_lidar_track_max_age_s = LaunchConfiguration(
        'obstacle_lidar_track_max_age_s')
    obstacle_lidar_track_max_misses = LaunchConfiguration(
        'obstacle_lidar_track_max_misses')
    obstacle_lidar_track_max_distance_jump_m = LaunchConfiguration(
        'obstacle_lidar_track_max_distance_jump_m')
    obstacle_lidar_track_max_range_rate_mps = LaunchConfiguration(
        'obstacle_lidar_track_max_range_rate_mps')
    obstacle_lidar_track_max_total_distance_jump_m = LaunchConfiguration(
        'obstacle_lidar_track_max_total_distance_jump_m')
    obstacle_lidar_track_max_offset_jump_px = LaunchConfiguration(
        'obstacle_lidar_track_max_offset_jump_px')
    obstacle_lidar_track_max_offset_rate_px_s = LaunchConfiguration(
        'obstacle_lidar_track_max_offset_rate_px_s')
    obstacle_lidar_track_bbox_iou_min = LaunchConfiguration(
        'obstacle_lidar_track_bbox_iou_min')
    obstacle_lidar_track_bbox_center_gate_px = LaunchConfiguration(
        'obstacle_lidar_track_bbox_center_gate_px')
    obstacle_lidar_near_cluster_distance_m = LaunchConfiguration(
        'obstacle_lidar_near_cluster_distance_m')
    obstacle_lidar_near_cluster_min_bbox_height_ratio = LaunchConfiguration(
        'obstacle_lidar_near_cluster_min_bbox_height_ratio')
    obstacle_lidar_near_cluster_min_bbox_bottom_ratio = LaunchConfiguration(
        'obstacle_lidar_near_cluster_min_bbox_bottom_ratio')
    obstacle_camera_fallback_speed_enabled = LaunchConfiguration(
        'obstacle_camera_fallback_speed_enabled')
    obstacle_camera_fallback_min_bottom_ratio = LaunchConfiguration(
        'obstacle_camera_fallback_min_bottom_ratio')
    obstacle_allow_avoid_on_curve = LaunchConfiguration(
        'obstacle_allow_avoid_on_curve')
    dynamic_event_duration_s = LaunchConfiguration(
        'dynamic_event_duration_s')
    dynamic_speed_confirm_min_matches = LaunchConfiguration(
        'dynamic_speed_confirm_min_matches')
    dynamic_speed_confirm_window_frames = LaunchConfiguration(
        'dynamic_speed_confirm_window_frames')
    dynamic_avoid_confirm_min_matches = LaunchConfiguration(
        'dynamic_avoid_confirm_min_matches')
    dynamic_avoid_confirm_window_frames = LaunchConfiguration(
        'dynamic_avoid_confirm_window_frames')
    dynamic_rearm_clear_frames = LaunchConfiguration(
        'dynamic_rearm_clear_frames')
    dynamic_obstacle_speed = LaunchConfiguration('dynamic_obstacle_speed')
    dynamic_speed_trigger_distance_m = LaunchConfiguration(
        'dynamic_speed_trigger_distance_m')
    dynamic_overtake_trigger_distance_m = LaunchConfiguration(
        'dynamic_overtake_trigger_distance_m')
    dynamic_lane_determination_distance_m = LaunchConfiguration(
        'dynamic_lane_determination_distance_m')
    static_event_duration_s = LaunchConfiguration(
        'static_event_duration_s')
    static_speed_confirm_min_matches = LaunchConfiguration(
        'static_speed_confirm_min_matches')
    static_speed_confirm_window_frames = LaunchConfiguration(
        'static_speed_confirm_window_frames')
    static_avoid_confirm_min_matches = LaunchConfiguration(
        'static_avoid_confirm_min_matches')
    static_avoid_confirm_window_frames = LaunchConfiguration(
        'static_avoid_confirm_window_frames')
    static_rearm_clear_frames = LaunchConfiguration(
        'static_rearm_clear_frames')
    static_obstacle_speed = LaunchConfiguration('static_obstacle_speed')
    static_speed_trigger_distance_m = LaunchConfiguration(
        'static_speed_trigger_distance_m')
    static_overtake_trigger_distance_m = LaunchConfiguration(
        'static_overtake_trigger_distance_m')
    static_lane_determination_distance_m = LaunchConfiguration(
        'static_lane_determination_distance_m')
    lidar_rotation_deg = LaunchConfiguration('lidar_rotation_deg')
    cone_entry_fusion_rotation_deg = LaunchConfiguration(
        'cone_entry_fusion_rotation_deg')
    cone_entry_fusion_projection_model = LaunchConfiguration(
        'cone_entry_fusion_projection_model')
    cone_entry_use_fusion_gate = LaunchConfiguration(
        'cone_entry_use_fusion_gate')
    cone_entry_allow_lidar_fallback = LaunchConfiguration(
        'cone_entry_allow_lidar_fallback')
    cone_entry_min_fused_clusters = LaunchConfiguration(
        'cone_entry_min_fused_clusters')
    cone_entry_require_both_sides = LaunchConfiguration(
        'cone_entry_require_both_sides')
    cone_entry_side_min_abs_y_m = LaunchConfiguration(
        'cone_entry_side_min_abs_y_m')
    cone_entry_fusion_timeout_s = LaunchConfiguration(
        'cone_entry_fusion_timeout_s')
    cone_entry_fusion_max_range_m = LaunchConfiguration(
        'cone_entry_fusion_max_range_m')
    cone_prearm_enabled = LaunchConfiguration('cone_prearm_enabled')
    cone_bbox_prearm_enabled = LaunchConfiguration(
        'cone_bbox_prearm_enabled')
    cone_prearm_use_fusion = LaunchConfiguration(
        'cone_prearm_use_fusion')
    cone_prearm_max_range_m = LaunchConfiguration(
        'cone_prearm_max_range_m')
    cone_prearm_min_bbox_area_px = LaunchConfiguration(
        'cone_prearm_min_bbox_area_px')
    cone_prearm_min_bottom_y_px = LaunchConfiguration(
        'cone_prearm_min_bottom_y_px')
    cone_bbox_prearm_min_detections = LaunchConfiguration(
        'cone_bbox_prearm_min_detections')
    cone_bbox_prearm_clear_frames = LaunchConfiguration(
        'cone_bbox_prearm_clear_frames')
    cone_prearm_min_clusters = LaunchConfiguration(
        'cone_prearm_min_clusters')
    cone_prearm_window = LaunchConfiguration('cone_prearm_window')
    cone_prearm_min_matches = LaunchConfiguration(
        'cone_prearm_min_matches')
    cone_prearm_hold_s = LaunchConfiguration('cone_prearm_hold_s')
    cone_prearm_speed_cap = LaunchConfiguration(
        'cone_prearm_speed_cap')
    cone_prearm_dbscan_eps_m = LaunchConfiguration(
        'cone_prearm_dbscan_eps_m')
    cone_prearm_dbscan_min_samples = LaunchConfiguration(
        'cone_prearm_dbscan_min_samples')
    cone_suspect_min_fused_clusters = LaunchConfiguration(
        'cone_suspect_min_fused_clusters')
    cone_suspect_hold_s = LaunchConfiguration('cone_suspect_hold_s')
    cone_suspect_speed_cap = LaunchConfiguration(
        'cone_suspect_speed_cap')
    cone_perception_mode = LaunchConfiguration('cone_perception_mode')
    lidar_cluster_min_points = LaunchConfiguration(
        'lidar_cluster_min_points')
    lidar_cluster_min_chord_m = LaunchConfiguration(
        'lidar_cluster_min_chord_m')
    lidar_cluster_max_radius_m = LaunchConfiguration(
        'lidar_cluster_max_radius_m')
    lidar_cluster_max_chord_m = LaunchConfiguration(
        'lidar_cluster_max_chord_m')
    lidar_association_distance_m = LaunchConfiguration(
        'lidar_association_distance_m')
    lidar_min_track_hits = LaunchConfiguration('lidar_min_track_hits')
    lidar_max_track_misses = LaunchConfiguration('lidar_max_track_misses')
    lidar_min_entry_pairs = LaunchConfiguration('lidar_min_entry_pairs')
    lidar_entry_min_clusters = LaunchConfiguration(
        'lidar_entry_min_clusters')
    lidar_min_pair_span_m = LaunchConfiguration('lidar_min_pair_span_m')
    lidar_max_entry_width_spread_m = LaunchConfiguration(
        'lidar_max_entry_width_spread_m')
    lidar_allow_cone_reentry = LaunchConfiguration(
        'lidar_allow_cone_reentry')
    lidar_boundary_tolerance_m = LaunchConfiguration(
        'lidar_boundary_tolerance_m')
    lidar_min_one_side_points = LaunchConfiguration(
        'lidar_min_one_side_points')
    lidar_two_point_support_confirm_frames = LaunchConfiguration(
        'lidar_two_point_support_confirm_frames')
    lidar_established_hold_frames = LaunchConfiguration(
        'lidar_established_hold_frames')
    lidar_dbscan_entry_min_clusters = LaunchConfiguration(
        'lidar_dbscan_entry_min_clusters')
    lidar_dbscan_allow_cone_reentry = LaunchConfiguration(
        'lidar_dbscan_allow_cone_reentry')
    lidar_dbscan_min_range_m = LaunchConfiguration(
        'lidar_dbscan_min_range_m')
    lidar_dbscan_max_range_m = LaunchConfiguration(
        'lidar_dbscan_max_range_m')
    lidar_dbscan_eps_m = LaunchConfiguration('lidar_dbscan_eps_m')
    lidar_dbscan_min_samples = LaunchConfiguration(
        'lidar_dbscan_min_samples')
    lidar_dbscan_max_cone_diameter_m = LaunchConfiguration(
        'lidar_dbscan_max_cone_diameter_m')
    lidar_dbscan_angle_bin_deg = LaunchConfiguration(
        'lidar_dbscan_angle_bin_deg')
    lidar_dbscan_min_cluster_separation_m = LaunchConfiguration(
        'lidar_dbscan_min_cluster_separation_m')
    lane_straight_speed = LaunchConfiguration('lane_straight_speed')
    lane_curve_strong_speed = LaunchConfiguration(
        'lane_curve_strong_speed')
    lane_curve_medium_speed = LaunchConfiguration(
        'lane_curve_medium_speed')
    lane_curve_mild_speed = LaunchConfiguration('lane_curve_mild_speed')
    lane_straight_recovery_speed = LaunchConfiguration(
        'lane_straight_recovery_speed')
    lane_curve_strong_threshold = LaunchConfiguration(
        'lane_curve_strong_threshold')
    lane_curve_medium_threshold = LaunchConfiguration(
        'lane_curve_medium_threshold')
    lane_curve_accel_rate = LaunchConfiguration(
        'lane_curve_accel_rate')
    lane_straight_accel_rate = LaunchConfiguration(
        'lane_straight_accel_rate')
    lane_curve_decel_rate = LaunchConfiguration(
        'lane_curve_decel_rate')
    lane_straight_decel_rate = LaunchConfiguration(
        'lane_straight_decel_rate')
    lane_straight_enter_cte_px = LaunchConfiguration(
        'lane_straight_enter_cte_px')
    lane_straight_exit_cte_px = LaunchConfiguration(
        'lane_straight_exit_cte_px')
    lane_straight_enter_heading_deg = LaunchConfiguration(
        'lane_straight_enter_heading_deg')
    lane_straight_exit_heading_deg = LaunchConfiguration(
        'lane_straight_exit_heading_deg')
    lane_straight_enter_curvature_risk = LaunchConfiguration(
        'lane_straight_enter_curvature_risk')
    lane_straight_exit_curvature_risk = LaunchConfiguration(
        'lane_straight_exit_curvature_risk')
    lane_curve_confirmation_frames = LaunchConfiguration(
        'lane_curve_confirmation_frames')
    lane_straight_confirmation_frames = LaunchConfiguration(
        'lane_straight_confirmation_frames')
    lane_curvature_filter_window = LaunchConfiguration(
        'lane_curvature_filter_window')
    lane_curvature_scale = LaunchConfiguration('lane_curvature_scale')
    stanley_control_rate_hz = LaunchConfiguration(
        'stanley_control_rate_hz')
    mission_decision_rate_hz = LaunchConfiguration(
        'mission_decision_rate_hz')
    stanley_state_timeout_s = LaunchConfiguration(
        'stanley_state_timeout_s')
    stanley_hold_last_angle_when_stale = LaunchConfiguration(
        'stanley_hold_last_angle_when_stale')
    stanley_curvature_steering_rate_enabled = LaunchConfiguration(
        'stanley_curvature_steering_rate_enabled')
    stanley_steering_rate_straight_per_s = LaunchConfiguration(
        'stanley_steering_rate_straight_per_s')
    stanley_steering_rate_curve_per_s = LaunchConfiguration(
        'stanley_steering_rate_curve_per_s')
    stanley_steering_rate_reversal_per_s = LaunchConfiguration(
        'stanley_steering_rate_reversal_per_s')
    stanley_steering_rate_risk_low = LaunchConfiguration(
        'stanley_steering_rate_risk_low')
    stanley_steering_rate_risk_high = LaunchConfiguration(
        'stanley_steering_rate_risk_high')
    stanley_steering_rate_hold_frames = LaunchConfiguration(
        'stanley_steering_rate_hold_frames')
    stanley_image_center_x = LaunchConfiguration('stanley_image_center_x')
    stanley_gain_schedule_start_speed = LaunchConfiguration(
        'stanley_gain_schedule_start_speed')
    stanley_gain_schedule_end_speed = LaunchConfiguration(
        'stanley_gain_schedule_end_speed')
    stanley_k_straight_low_speed = LaunchConfiguration(
        'stanley_k_straight_low_speed')
    stanley_k_straight_high_speed = LaunchConfiguration(
        'stanley_k_straight_high_speed')
    stanley_k_curve_low_speed = LaunchConfiguration(
        'stanley_k_curve_low_speed')
    stanley_k_curve_high_speed = LaunchConfiguration(
        'stanley_k_curve_high_speed')
    stanley_k_obstacle = LaunchConfiguration('stanley_k_obstacle')
    stanley_adaptive_heading_weight_enabled = LaunchConfiguration(
        'stanley_adaptive_heading_weight_enabled')
    stanley_heading_weight_low_speed = LaunchConfiguration(
        'stanley_heading_weight_low_speed')
    stanley_heading_weight_straight_high_speed = LaunchConfiguration(
        'stanley_heading_weight_straight_high_speed')
    stanley_heading_weight_hough_high_speed = LaunchConfiguration(
        'stanley_heading_weight_hough_high_speed')
    stanley_heading_weight_curve_high_speed = LaunchConfiguration(
        'stanley_heading_weight_curve_high_speed')
    stanley_heading_weight_rise_alpha = LaunchConfiguration(
        'stanley_heading_weight_rise_alpha')
    stanley_heading_weight_fall_alpha = LaunchConfiguration(
        'stanley_heading_weight_fall_alpha')
    stanley_accel_rate_per_s = LaunchConfiguration(
        'stanley_accel_rate_per_s')
    stanley_decel_rate_per_s = LaunchConfiguration(
        'stanley_decel_rate_per_s')
    cone_controller = LaunchConfiguration('cone_controller')
    cone_min_speed = LaunchConfiguration('cone_min_speed')
    cone_max_speed = LaunchConfiguration('cone_max_speed')
    cone_speed_accel_rate_per_s = LaunchConfiguration(
        'cone_speed_accel_rate_per_s')
    cone_speed_decel_rate_per_s = LaunchConfiguration(
        'cone_speed_decel_rate_per_s')
    cone_stanley_control_x_m = LaunchConfiguration(
        'cone_stanley_control_x_m')
    cone_stanley_heading_window_m = LaunchConfiguration(
        'cone_stanley_heading_window_m')
    cone_stanley_cross_track_gain = LaunchConfiguration(
        'cone_stanley_cross_track_gain')
    cone_stanley_heading_gain = LaunchConfiguration(
        'cone_stanley_heading_gain')
    cone_stanley_softening_speed_mps = LaunchConfiguration(
        'cone_stanley_softening_speed_mps')
    cone_stanley_max_angle_step = LaunchConfiguration(
        'cone_stanley_max_angle_step')
    cone_stanley_smoothing_alpha = LaunchConfiguration(
        'cone_stanley_smoothing_alpha')
    cone_stanley_path_timeout_s = LaunchConfiguration(
        'cone_stanley_path_timeout_s')

    max_range_m = LaunchConfiguration('max_range_m')
    x_max_m = LaunchConfiguration('x_max_m')
    y_abs_max_m = LaunchConfiguration('y_abs_max_m')
    gap_min_width_m = LaunchConfiguration('gap_min_width_m')
    gap_max_width_m = LaunchConfiguration('gap_max_width_m')
    gap_expected_width_m = LaunchConfiguration('gap_expected_width_m')
    cone_spacing_m = LaunchConfiguration('cone_spacing_m')
    cone_one_side_half_width_m = LaunchConfiguration(
        'cone_one_side_half_width_m')
    cone_pair_min_normal_alignment = LaunchConfiguration(
        'cone_pair_min_normal_alignment')
    cone_pair_max_longitudinal_m = LaunchConfiguration(
        'cone_pair_max_longitudinal_m')
    cone_midpoint_reference_gate_m = LaunchConfiguration(
        'cone_midpoint_reference_gate_m')
    cone_min_midpoints_for_path = LaunchConfiguration(
        'cone_min_midpoints_for_path')
    cone_min_path_span_m = LaunchConfiguration('cone_min_path_span_m')
    cone_max_path_lateral_jump_m = LaunchConfiguration(
        'cone_max_path_lateral_jump_m')
    cone_path_lost_keep_frames = LaunchConfiguration(
        'cone_path_lost_keep_frames')
    cone_path_smoothing_alpha = LaunchConfiguration(
        'cone_path_smoothing_alpha')
    bbox_padding_x_px = LaunchConfiguration('bbox_padding_x_px')
    bbox_padding_y_px = LaunchConfiguration('bbox_padding_y_px')
    cone_entry_min_detections = LaunchConfiguration(
        'cone_entry_min_detections')
    cone_entry_min_path_points = LaunchConfiguration(
        'cone_entry_min_path_points')
    cone_enter_confirm_cycles = LaunchConfiguration(
        'cone_enter_confirm_cycles')
    cone_entry_min_bottom_y_px = LaunchConfiguration(
        'cone_entry_min_bottom_y_px')
    cone_exit_hold_s = LaunchConfiguration('cone_exit_hold_s')
    cone_min_mode_duration_s = LaunchConfiguration(
        'cone_min_mode_duration_s')

    any_perception_pipeline_enabled = PythonExpression([
        "'", enable_lane_pipeline, "'.lower() == 'true' or '",
        enable_scene_pipeline, "'.lower() == 'true'",
    ])
    lane_yolo_enabled = PythonExpression([
        "'", start_yolo, "'.lower() == 'true' and '",
        enable_lane_pipeline, "'.lower() == 'true'",
    ])
    scene_yolo_enabled = PythonExpression([
        "'", start_yolo, "'.lower() == 'true' and '",
        enable_scene_pipeline, "'.lower() == 'true'",
    ])
    lane_stack_enabled = PythonExpression([
        "'", start_lane_stack, "'.lower() == 'true' and '",
        enable_lane_pipeline, "'.lower() == 'true'",
    ])
    shortcut_left_turn_enabled = PythonExpression([
        "'", start_lane_stack, "'.lower() == 'true' and '",
        enable_lane_pipeline, "'.lower() == 'true' and '",
        enable_shortcut_left_turn, "'.lower() == 'true'",
    ])
    center_curve_output_topic = PythonExpression([
        "'/center_curve/base' if '",
        enable_shortcut_left_turn,
        "'.lower() == 'true' else '/center_curve'",
    ])
    lane_control_enabled = PythonExpression([
        "'", start_lane_stack, "'.lower() == 'true' and '",
        enable_lane_pipeline, "'.lower() == 'true' and '",
        enable_drive_control, "'.lower() == 'true'",
    ])
    cone_drive_enabled = PythonExpression([
        "'", start_lane_stack, "'.lower() == 'true' and '",
        enable_lane_pipeline, "'.lower() == 'true' and '",
        enable_scene_pipeline, "'.lower() == 'true' and '",
        enable_drive_control, "'.lower() == 'true'",
    ])
    lidar_driver_enabled = PythonExpression([
        "'", start_lidar, "'.lower() == 'true' and '",
        start_lane_stack, "'.lower() == 'true' and '",
        enable_lane_pipeline, "'.lower() == 'true' and '",
        enable_scene_pipeline, "'.lower() == 'true' and '",
        enable_drive_control, "'.lower() == 'true'",
    ])
    ultrasonic_stack_enabled = PythonExpression([
        "'", start_ultrasonic, "'.lower() == 'true' and '",
        start_lane_stack, "'.lower() == 'true' and '",
        enable_lane_pipeline, "'.lower() == 'true' and '",
        enable_scene_pipeline, "'.lower() == 'true' and '",
        enable_drive_control, "'.lower() == 'true'",
    ])
    target_lane_planner_enabled = PythonExpression([
        "'", start_lane_stack, "'.lower() == 'true' and '",
        enable_lane_pipeline, "'.lower() == 'true' and '",
        enable_scene_pipeline, "'.lower() == 'true' and '",
        enable_drive_control, "'.lower() == 'true'",
    ])

    rviz_config = os.path.join(
        get_package_share_directory('mission_cone_drive'),
        'rviz',
        'cone_drive.rviz',
    )

    actions = [
        DeclareLaunchArgument(
            'execution_backend',
            default_value='auto',
            choices=['auto', 'host', 'container'],
            description=(
                'auto uses the host when CUDA Python dependencies exist and '
                'otherwise starts the maintained Jetson GPU container.'),
        ),
        DeclareLaunchArgument(
            'container_image',
            default_value='xycar-jetson:20260817-integrated',
            description=(
                'GPU runtime image used by execution_backend=container.')),
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument(
            'start_ultrasonic',
            default_value='false',
            description=(
                'Start the ultrasonic serial stack. False is the vehicle '
                'default because the current /dev/ttySonic udev alias '
                'collides with the CH340 used by VESC.'),
        ),
        DeclareLaunchArgument('start_yolo', default_value='true'),
        DeclareLaunchArgument(
            'mission_start_service_name',
            default_value='/start_integrated_drive',
            description=(
                'Mission Manager start service. The automatic container '
                'backend remaps this internally and exposes the same public '
                'service through its host proxy.'),
        ),
        DeclareLaunchArgument(
            'mission_stop_service_name',
            default_value='/stop_integrated_drive',
            description=(
                'Mission Manager stop service. The automatic container '
                'backend remaps this internally and exposes the same public '
                'service through its host proxy.'),
        ),
        DeclareLaunchArgument(
            'mission_pause_service_name',
            default_value='/pause_integrated_drive',
            description='Pause driving without clearing mission state.',
        ),
        DeclareLaunchArgument(
            'mission_resume_service_name',
            default_value='/resume_integrated_drive',
            description='Resume a paused drive after readiness checks.',
        ),
        DeclareLaunchArgument(
            'mission_toggle_pause_service_name',
            default_value='/toggle_pause_integrated_drive',
            description='Toggle between paused and running states.',
        ),
        DeclareLaunchArgument(
            'enable_spacebar_pause',
            default_value='true',
            description=(
                'Read spacebar from the launch terminal to toggle pause.')),
        DeclareLaunchArgument('start_lane_stack', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='false'),
        DeclareLaunchArgument('start_active', default_value='false'),
        DeclareLaunchArgument(
            'start_motor_driver',
            default_value='true',
            description=(
                'Start VESC UART and the /xycar_motor adapter inside this '
                'integrated launch. Motor output remains zero until the '
                '/start_integrated_drive service accepts the start request.'),
        ),
        DeclareLaunchArgument(
            'motor_port',
            default_value='auto',
            description=(
                'VESC UART path, or auto for ttyMOTOR -> matching by-id -> '
                'unambiguous ttyUSB selection.'),
        ),
        DeclareLaunchArgument(
            'motor_auto_device_symlink', default_value='/dev/ttyMOTOR'),
        DeclareLaunchArgument(
            'motor_auto_device_match', default_value='1a86_USB_Serial'),
        DeclareLaunchArgument(
            'vesc_handshake_timeout_s', default_value='2.0'),
        DeclareLaunchArgument(
            'vesc_reconnect_interval_s', default_value='1.0'),
        DeclareLaunchArgument(
            'vesc_telemetry_timeout_s', default_value='1.0'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('motor_topic', default_value='/xycar_motor'),
        DeclareLaunchArgument('show_yolo_view', default_value='false'),
        DeclareLaunchArgument('show_lane_view', default_value='false'),
        DeclareLaunchArgument('show_overtake_view', default_value='false'),
        DeclareLaunchArgument(
            'model_filename',
            default_value=(
                'all_second_yolo_v2_yolov10n_640_best_e46.pt'),
        ),
        DeclareLaunchArgument(
            'perception_image_topic',
            default_value='/perception/scene/image',
            description=(
                'Compatibility alias used as the default scene image topic.'
            ),
        ),
        DeclareLaunchArgument('cone_conf_threshold', default_value='0.15'),
        DeclareLaunchArgument(
            'yolo_max_rate_hz',
            default_value='5.0',
            description='Compatibility alias for scene_max_rate_hz.',
        ),
        DeclareLaunchArgument(
            'yolo_image_size',
            default_value='640',
            description='Compatibility alias for scene_inference_image_size.',
        ),
        DeclareLaunchArgument(
            'yolo_image_qos_reliability',
            default_value='reliable',
        ),
        DeclareLaunchArgument(
            'yolo_torch_threads',
            default_value='2',
            description='Compatibility alias for scene_torch_threads.',
        ),
        DeclareLaunchArgument(
            'yolo_torch_interop_threads', default_value='1'),
        DeclareLaunchArgument('yolo_opencv_threads', default_value='1'),

        DeclareLaunchArgument('enable_lane_pipeline', default_value='true'),
        DeclareLaunchArgument('enable_scene_pipeline', default_value='true'),
        DeclareLaunchArgument('enable_drive_control', default_value='true'),
        DeclareLaunchArgument(
            'enable_shortcut_left_turn',
            default_value='true',
            description=(
                'Route center curves through the event-driven left-turn '
                'planner. It is a transparent pass-through until LEFT.'
            ),
        ),
        DeclareLaunchArgument(
            'shortcut_auto_trigger_after_s',
            default_value='-1.0',
            description='Replay-only trigger; negative uses the raw signal.'),
        DeclareLaunchArgument(
            'shortcut_auto_trigger_state',
            default_value='entry',
            description='Replay trigger target: entry or exit.'),
        DeclareLaunchArgument(
            'shortcut_prepare_speed_limit', default_value='3.0'),
        DeclareLaunchArgument(
            'shortcut_turn_speed_limit', default_value='3.0'),
        DeclareLaunchArgument(
            'shortcut_cruise_speed_limit', default_value='-1.0'),
        DeclareLaunchArgument(
            'shortcut_steering_limit', default_value='39.0'),
        DeclareLaunchArgument(
            'shortcut_left_signal_window', default_value='5'),
        DeclareLaunchArgument(
            'shortcut_left_signal_min_matches', default_value='3'),
        DeclareLaunchArgument(
            'shortcut_fallback_commit_s', default_value='5.0'),
        DeclareLaunchArgument(
            'shortcut_entry_max_heading_deg', default_value='60.0'),
        DeclareLaunchArgument(
            'shortcut_min_cruise_duration_s', default_value='4.0'),
        DeclareLaunchArgument(
            'shortcut_crossline_window', default_value='3'),
        DeclareLaunchArgument(
            'shortcut_crossline_min_matches', default_value='2'),
        DeclareLaunchArgument(
            'shortcut_exit_min_turn_duration_s',
            default_value='1.5',
            description=(
                'Seconds to keep following the shortcut exit-left path '
                'before returning control to the normal lane path.'),
        ),
        DeclareLaunchArgument(
            'shortcut_exit_alignment_window', default_value='5'),
        DeclareLaunchArgument(
            'shortcut_exit_alignment_min_matches', default_value='3'),
        DeclareLaunchArgument(
            'shortcut_exit_handoff_duration_s', default_value='0.8'),
        DeclareLaunchArgument(
            'shortcut_exit_max_heading_deg', default_value='22.0'),
        DeclareLaunchArgument(
            'input_image_topic', default_value='/image_raw'),
        DeclareLaunchArgument(
            'camera_qos_reliability', default_value='reliable'),
        DeclareLaunchArgument(
            'publish_performance_stats',
            default_value='true',
            description=(
                'Log frame-router and YOLO input/output/inference/queue/'
                'source '
                'age statistics at performance_log_period_s.'),
        ),
        DeclareLaunchArgument(
            'performance_log_period_s', default_value='5.0'),
        DeclareLaunchArgument(
            'publish_pipeline_timing',
            default_value='false',
            description=(
                'Publish passive per-frame pipeline timestamps; no control '
                'input or output is changed.'),
        ),
        DeclareLaunchArgument(
            'pipeline_timing_topic', default_value='/pipeline_timing'),
        DeclareLaunchArgument(
            'publish_resource_stats',
            default_value='false',
            description=(
                'Write passive 2-5 Hz CPU/GPU samples to CSV. Disabled by '
                'default and independent from pipeline timing.'),
        ),
        DeclareLaunchArgument(
            'resource_stats_rate_hz', default_value='2.0'),
        DeclareLaunchArgument(
            'resource_stats_output_path',
            default_value='/tmp/xycar_resource_stats.csv'),
        DeclareLaunchArgument(
            'resource_stats_process_pattern',
            default_value='lane_yolo_node'),
        DeclareLaunchArgument(
            'resource_stats_gpu_index', default_value='0'),
        DeclareLaunchArgument(
            'resource_stats_profile_threads',
            default_value='false',
            description=(
                'Collect per-thread target CPU rows. Keep false for normal '
                'A/B; enable only for a dedicated scheduling diagnosis.'),
        ),
        DeclareLaunchArgument(
            'stanley_event_driven',
            default_value='false',
            description=(
                'Compute once immediately for each new lane state while '
                'retaining the control timer as a heartbeat.'),
        ),
        DeclareLaunchArgument(
            'mission_event_driven_lane',
            default_value='false',
            description=(
                'Immediately forward only arbitration-safe fresh lane '
                'commands; decision timer remains the heartbeat.'),
        ),
        DeclareLaunchArgument(
            'publish_camera_timing',
            default_value='false',
            description='Enable passive usb_cam T0..T7 profiling.'),
        DeclareLaunchArgument(
            'camera_timing_topic', default_value='/camera_timing'),
        DeclareLaunchArgument(
            'future_stamp_tolerance_s', default_value='0.05'),

        DeclareLaunchArgument(
            'lane_model_filename',
            default_value='center_line_yolov10n_320_best.pt'),
        DeclareLaunchArgument(
            'lane_image_topic', default_value='/perception/lane/image'),
        DeclareLaunchArgument('lane_image_width', default_value='320'),
        DeclareLaunchArgument('lane_image_height', default_value='240'),
        DeclareLaunchArgument(
            'lane_max_rate_hz',
            default_value='12.0',
            description=(
                'Maximum lane-image rate from frame_router (Hz); this is an '
                'upstream ceiling, not the YOLO inference limiter.'),
        ),
        DeclareLaunchArgument(
            'frame_router_event_driven',
            default_value='false',
            description=(
                'Arrival-driven latest-frame lane routing; timer mode remains '
                'the default for isolated A/B testing.'),
        ),
        DeclareLaunchArgument(
            'lane_yolo_max_rate_hz',
            default_value='0.0',
            description=(
                'Lane YOLO inference cap in Hz; 0 means no additional cap. '
                'Actual rate cannot exceed lane_max_rate_hz.'),
        ),
        DeclareLaunchArgument(
            'lane_inference_image_size', default_value='320'),
        DeclareLaunchArgument(
            'lane_class_names', default_value='center_line'),
        DeclareLaunchArgument('lane_torch_threads', default_value='2'),
        DeclareLaunchArgument(
            'lane_torch_interop_threads', default_value='1'),
        DeclareLaunchArgument('lane_opencv_threads', default_value='1'),
        DeclareLaunchArgument(
            'lane_torch_compile_mode', default_value='none'),
        DeclareLaunchArgument(
            'lane_compile_warmup_iterations', default_value='2'),
        DeclareLaunchArgument(
            'lane_yolo_device',
            default_value='',
            description='Ultralytics device; empty preserves auto selection.'),
        DeclareLaunchArgument(
            'lane_yolo_fp16',
            default_value='false',
            description=(
                'Request real CUDA FP16 for lane YOLO. Startup fails instead '
                'of silently falling back to CPU/FP32.'),
        ),
        DeclareLaunchArgument(
            'lane_yolo_runtime_report_path',
            default_value='',
            description=(
                'Optional JSON proving actual backend/device/model/input '
                'dtype and warm-up timing.'),
        ),
        DeclareLaunchArgument('lane_process_nice', default_value='0'),
        DeclareLaunchArgument(
            'lane_cpu_affinity',
            default_value='0-3',
            description=(
                'Logical CPUs reserved for lane YOLO on Jetson Orin NX.'
            ),
        ),
        DeclareLaunchArgument(
            'lane_image_qos_reliability', default_value='reliable'),
        DeclareLaunchArgument(
            'lane_annotated_max_rate_hz', default_value='6.0'),
        DeclareLaunchArgument(
            'show_lane_yolo_view', default_value=show_yolo_view),
        DeclareLaunchArgument(
            'show_lane_processing_view', default_value=show_lane_view),
        DeclareLaunchArgument('lane_base_input_width', default_value='640'),
        DeclareLaunchArgument('lane_base_input_height', default_value='480'),
        DeclareLaunchArgument(
            'lane_center_height_threshold_px', default_value='220'),
        DeclareLaunchArgument(
            'lane_center_adaptive_block_size_px', default_value='13'),
        DeclareLaunchArgument(
            'lane_center_hough_threshold', default_value='20'),
        DeclareLaunchArgument(
            'lane_center_hough_min_line_length_px', default_value='20'),
        DeclareLaunchArgument(
            'lane_center_hough_max_line_gap_px', default_value='5'),
        DeclareLaunchArgument(
            'lane_center_similarity_threshold_px', default_value='70'),
        DeclareLaunchArgument(
            'lane_center_rmse_threshold_px', default_value='30'),
        DeclareLaunchArgument(
            'lane_center_bottom_box_height_threshold_px',
            default_value='10',
        ),
        DeclareLaunchArgument(
            'lane_center_bottom_point_height_diff_threshold_px',
            default_value='5',
        ),
        DeclareLaunchArgument(
            'lane_center_min_bbox_width_px', default_value='5'),
        DeclareLaunchArgument(
            'lane_center_min_bbox_height_px', default_value='5'),
        DeclareLaunchArgument(
            'lane_center_x_distance_threshold_px', default_value='300'),
        DeclareLaunchArgument(
            'lane_center_point_merge_distance_px', default_value='2.0'),
        DeclareLaunchArgument(
            'lane_center_spline_smoothing_px2', default_value='30.0'),
        DeclareLaunchArgument(
            'lane_canny_blur_kernel_px', default_value='7'),
        DeclareLaunchArgument(
            'lane_object_grid_cell_size_px', default_value='50'),
        DeclareLaunchArgument(
            'lane_curve_heading_recovery_enabled', default_value='true'),
        DeclareLaunchArgument(
            'lane_curve_heading_disagreement_deg', default_value='25.0'),
        DeclareLaunchArgument(
            'lane_curve_heading_recovery_weight', default_value='0.65'),
        DeclareLaunchArgument(
            'lane_curve_heading_confirm_frames', default_value='2'),
        DeclareLaunchArgument(
            'lane_curve_heading_curve_max_fit_error_ratio',
            default_value='0.15',
        ),
        DeclareLaunchArgument(
            'lane_curve_heading_extrapolation_margin_ratio',
            default_value='0.75',
        ),
        DeclareLaunchArgument(
            'lane_adaptive_heading_enabled', default_value='true'),
        DeclareLaunchArgument(
            'lane_heading_preview_min_ratio', default_value='0.08'),
        DeclareLaunchArgument(
            'lane_heading_preview_max_ratio', default_value='0.16'),
        DeclareLaunchArgument(
            'lane_heading_curvature_scale', default_value='0.50'),
        DeclareLaunchArgument(
            'lane_heading_curvature_deadzone', default_value='0.0'),
        DeclareLaunchArgument(
            'lane_heading_filter_alpha_straight', default_value='0.65'),
        DeclareLaunchArgument(
            'lane_heading_filter_alpha_curve', default_value='0.70'),
        DeclareLaunchArgument(
            'lane_heading_filter_alpha_fallback', default_value='0.35'),
        DeclareLaunchArgument(
            'lane_heading_curve_hold_frames', default_value='3'),
        DeclareLaunchArgument(
            'lane_heading_max_step_deg', default_value='12.0'),
        DeclareLaunchArgument(
            'lane_curve_center_agreement_threshold_px',
            default_value='40.0',
            description=(
                'Center-curve correction threshold for normal Curve driving'),
        ),
        DeclareLaunchArgument(
            'lane_curve_center_correction_weight',
            default_value='0.85',
            description=(
                'Center-curve weight for normal Curve driving'),
        ),
        DeclareLaunchArgument(
            'lane_curve_center_hold_frames',
            default_value='3',
            description=(
                'Source frames to retain the last valid Curve center'),
        ),
        DeclareLaunchArgument(
            'lane_straight_center_agreement_threshold_px',
            default_value='10.0',
            description=(
                'YOLO correction threshold for Straight/recovery driving'),
        ),
        DeclareLaunchArgument(
            'lane_straight_center_correction_weight',
            default_value='1.0',
            description='YOLO center weight for Straight/recovery driving',
        ),
        DeclareLaunchArgument(
            'lane_straight_center_max_jump_px',
            default_value='120.0',
            description='Single-frame YOLO center jump requiring confirmation',
        ),
        DeclareLaunchArgument(
            'lane_straight_center_jump_confirm_frames',
            default_value='2',
            description='Frames required to accept a large YOLO center jump',
        ),
        DeclareLaunchArgument(
            'lane_obstacle_recovery_duration_s',
            default_value='3.0',
            description='YOLO-priority duration after a lane override reset',
        ),
        DeclareLaunchArgument(
            'lane_lateral_innovation_straight_px',
            default_value='35.0',
            description='Straight Hough center innovation gate in BEV pixels',
        ),
        DeclareLaunchArgument(
            'lane_lateral_innovation_curve_px',
            default_value='120.0',
            description='Fully relaxed curve/S innovation gate in BEV pixels',
        ),
        DeclareLaunchArgument(
            'lane_lateral_max_rate_straight_px_s',
            default_value='320.0',
            description='Straight lateral safety rate in BEV pixels/second',
        ),
        DeclareLaunchArgument(
            'lane_lateral_lane_half_width_px',
            default_value='250.0',
            description='Expected center-to-outer-lane spacing in BEV pixels',
        ),
        DeclareLaunchArgument(
            'lane_lateral_outer_pair_tolerance_px',
            default_value='80.0',
            description='Allowed full outer-pair width error in BEV pixels',
        ),
        DeclareLaunchArgument(
            'lane_lateral_outer_pair_hold_frames',
            default_value='5',
            description=(
                'Straight frames to retain a missing outer-pair midpoint'),
        ),

        DeclareLaunchArgument(
            'scene_model_filename',
            default_value=model_filename),
        DeclareLaunchArgument(
            'scene_image_topic', default_value=LaunchConfiguration(
                'perception_image_topic')),
        DeclareLaunchArgument('scene_image_width', default_value='640'),
        DeclareLaunchArgument('scene_image_height', default_value='480'),
        DeclareLaunchArgument(
            'scene_max_rate_hz',
            default_value=LaunchConfiguration('yolo_max_rate_hz'),
            description=(
                'Maximum scene-image rate from frame_router (Hz); '
                'yolo_max_rate_hz remains its compatibility default.'),
        ),
        DeclareLaunchArgument(
            'scene_yolo_max_rate_hz',
            default_value='0.0',
            description=(
                'Scene YOLO inference cap in Hz; 0 means no additional cap. '
                'Actual rate cannot exceed scene_max_rate_hz.'),
        ),
        DeclareLaunchArgument(
            'scene_inference_image_size', default_value=LaunchConfiguration(
                'yolo_image_size')),
        DeclareLaunchArgument(
            'scene_class_names',
            default_value='cone,dynamic,green,left,red,static',
        ),
        DeclareLaunchArgument(
            'scene_class_name_aliases',
            default_value='dynamic=obstacle_vehicle',
        ),
        DeclareLaunchArgument(
            'scene_general_conf_threshold', default_value='0.20'),
        DeclareLaunchArgument(
            'scene_obstacle_conf_threshold', default_value='0.35'),
        DeclareLaunchArgument(
            'scene_enable_checkerboard', default_value='false'),
        DeclareLaunchArgument('scene_torch_threads', default_value='1'),
        DeclareLaunchArgument(
            'scene_torch_interop_threads',
            default_value=LaunchConfiguration(
                'yolo_torch_interop_threads')),
        DeclareLaunchArgument(
            'scene_opencv_threads',
            default_value=LaunchConfiguration('yolo_opencv_threads')),
        DeclareLaunchArgument(
            'scene_torch_compile_mode', default_value='none'),
        DeclareLaunchArgument(
            'scene_compile_warmup_iterations', default_value='2'),
        DeclareLaunchArgument(
            'scene_yolo_device',
            default_value='',
            description='Ultralytics device; empty preserves auto selection.'),
        DeclareLaunchArgument(
            'scene_yolo_fp16',
            default_value='false',
            description=(
                'Request real CUDA FP16 for scene YOLO. Startup fails instead '
                'of silently falling back to CPU/FP32.'),
        ),
        DeclareLaunchArgument(
            'scene_yolo_runtime_report_path',
            default_value='',
            description=(
                'Optional JSON proving actual backend/device/model/input '
                'dtype and warm-up timing.'),
        ),
        DeclareLaunchArgument(
            'scene_process_nice',
            default_value=PythonExpression([
                "'10' if '",
                enable_lane_pipeline,
                "'.lower() == 'true' else '0'",
            ]),
            description=(
                'Scene OS nice value: defaults to 10 with lane enabled and '
                '0 in scene-only mode.'
            ),
        ),
        DeclareLaunchArgument(
            'scene_cpu_affinity',
            default_value=PythonExpression([
                "'4-7' if '",
                enable_lane_pipeline,
                "'.lower() == 'true' else '0-7'",
            ]),
            description=(
                'Logical CPUs for scene YOLO. Orin NX defaults keep dual '
                'YOLO worker pools on separate CPU groups.'
            ),
        ),
        DeclareLaunchArgument(
            'scene_image_qos_reliability',
            default_value=yolo_image_qos_reliability,
        ),
        DeclareLaunchArgument(
            'scene_detection_max_age_s', default_value='0.60'),
        DeclareLaunchArgument(
            'scene_annotated_max_rate_hz', default_value='4.0'),
        DeclareLaunchArgument(
            'show_scene_yolo_view', default_value=show_yolo_view),
        DeclareLaunchArgument(
            'show_traffic_light_view', default_value='false'),
        DeclareLaunchArgument(
            'traffic_direct_state_min_confidence', default_value='0.20'),
        DeclareLaunchArgument(
            'traffic_direct_red_min_confidence', default_value='0.20'),
        DeclareLaunchArgument(
            'traffic_direct_green_min_confidence', default_value='0.35'),
        DeclareLaunchArgument(
            'traffic_direct_left_min_confidence', default_value='0.50'),
        DeclareLaunchArgument(
            'traffic_stable_frames', default_value='2'),
        DeclareLaunchArgument(
            'traffic_left_is_green', default_value='true'),
        DeclareLaunchArgument(
            'traffic_red_latch_until_green',
            default_value='true',
            description=(
                'Keep a confirmed red stop until stable green/left. Set false '
                'for immediate runtime rollback to the timed hold.'),
        ),
        DeclareLaunchArgument(
            'traffic_red_latch_confirm_frames',
            default_value='2',
            description=(
                'Consecutive red detector samples required to arm the latch. '
                'The first direct red still stops immediately.'),
        ),
        DeclareLaunchArgument('sync_slop', default_value='0.20'),
        DeclareLaunchArgument('sync_queue_size', default_value='5'),
        DeclareLaunchArgument(
            'overtake_sensor_timeout_s', default_value='0.30'),
        DeclareLaunchArgument(
            'road_curve_max_time_delta_s',
            default_value='0.25',
            description=(
                'Maximum source-time offset for the physical-road curve '
                'state.')),
        DeclareLaunchArgument(
            'obstacle_lidar_min_range_m', default_value='0.18'),
        DeclareLaunchArgument(
            'obstacle_lidar_max_range_m', default_value='5.0'),
        DeclareLaunchArgument(
            'obstacle_lidar_cluster_min_points', default_value='2'),
        DeclareLaunchArgument(
            'obstacle_lidar_cluster_base_gap_m', default_value='0.08'),
        DeclareLaunchArgument(
            'obstacle_lidar_cluster_range_gap_scale', default_value='1.5'),
        DeclareLaunchArgument(
            'obstacle_lidar_cluster_max_width_m', default_value='2.0'),
        DeclareLaunchArgument(
            'obstacle_lidar_distance_percentile',
            default_value='50.0',
            description=(
                'Percentile of matched cluster ranges used as distance.')),
        DeclareLaunchArgument(
            'obstacle_lidar_association_padding_px', default_value='35.0'),
        DeclareLaunchArgument(
            'obstacle_lidar_association_max_above_px',
            default_value='10.0',
            description=(
                'Reject clusters projected this far above the YOLO box.')),
        DeclareLaunchArgument(
            'obstacle_lidar_camera_max_delta_s',
            default_value='0.15',
            description=(
                'Maximum camera-to-LiDAR source timestamp difference.')),
        DeclareLaunchArgument(
            'obstacle_lidar_tracking_enabled',
            default_value='true',
            description=(
                'Track the same YOLO/LiDAR object across perception frames.')),
        DeclareLaunchArgument(
            'obstacle_lidar_track_max_age_s', default_value='0.60'),
        DeclareLaunchArgument(
            'obstacle_lidar_track_max_misses', default_value='3'),
        DeclareLaunchArgument(
            'obstacle_lidar_track_max_distance_jump_m',
            default_value='0.45',
            description=(
                'Base allowed range change between tracked observations.')),
        DeclareLaunchArgument(
            'obstacle_lidar_track_max_range_rate_mps',
            default_value='3.0',
            description=(
                'Additional allowed tracked range change per elapsed '
                'second.')),
        DeclareLaunchArgument(
            'obstacle_lidar_track_max_total_distance_jump_m',
            default_value='1.50',
            description=(
                'Absolute maximum range change for one tracked update; '
                'zero disables this hard cap.')),
        DeclareLaunchArgument(
            'obstacle_lidar_track_max_offset_jump_px', default_value='55.0'),
        DeclareLaunchArgument(
            'obstacle_lidar_track_max_offset_rate_px_s',
            default_value='120.0'),
        DeclareLaunchArgument(
            'obstacle_lidar_track_bbox_iou_min', default_value='0.10'),
        DeclareLaunchArgument(
            'obstacle_lidar_track_bbox_center_gate_px',
            default_value='150.0'),
        DeclareLaunchArgument(
            'obstacle_lidar_near_cluster_distance_m', default_value='0.0'),
        DeclareLaunchArgument(
            'obstacle_lidar_near_cluster_min_bbox_height_ratio',
            default_value='0.16'),
        DeclareLaunchArgument(
            'obstacle_lidar_near_cluster_min_bbox_bottom_ratio',
            default_value='0.56'),
        DeclareLaunchArgument(
            'obstacle_camera_fallback_speed_enabled',
            default_value='true',
            description=(
                'Use a confirmed low YOLO box for an early speed cap while '
                'waiting for a LiDAR-cluster association.')),
        DeclareLaunchArgument(
            'obstacle_camera_fallback_min_bottom_ratio',
            default_value='0.48',
            description=(
                'Minimum bbox-bottom/image-height ratio for camera '
                'fallback.')),
        DeclareLaunchArgument(
            'obstacle_allow_avoid_on_curve',
            default_value='true',
            description=(
                'Allow confirmed obstacle lane-change events on curved '
                'road.')),
        DeclareLaunchArgument(
            'dynamic_event_duration_s', default_value='5.0'),
        DeclareLaunchArgument(
            'dynamic_speed_confirm_min_matches', default_value='2'),
        DeclareLaunchArgument(
            'dynamic_speed_confirm_window_frames', default_value='3'),
        DeclareLaunchArgument(
            'dynamic_avoid_confirm_min_matches', default_value='2'),
        DeclareLaunchArgument(
            'dynamic_avoid_confirm_window_frames', default_value='3'),
        DeclareLaunchArgument(
            'dynamic_rearm_clear_frames', default_value='5'),
        DeclareLaunchArgument(
            'dynamic_obstacle_speed',
            default_value='7.0',
            description='Dynamic-obstacle approach/avoid/return speed cap.'),
        DeclareLaunchArgument(
            'dynamic_speed_trigger_distance_m', default_value='10.0'),
        DeclareLaunchArgument(
            'dynamic_overtake_trigger_distance_m', default_value='4.5'),
        DeclareLaunchArgument(
            'dynamic_lane_determination_distance_m', default_value='5.0'),
        DeclareLaunchArgument(
            'static_event_duration_s', default_value='4.0'),
        DeclareLaunchArgument(
            'static_speed_confirm_min_matches', default_value='2'),
        DeclareLaunchArgument(
            'static_speed_confirm_window_frames', default_value='3'),
        DeclareLaunchArgument(
            'static_avoid_confirm_min_matches', default_value='2'),
        DeclareLaunchArgument(
            'static_avoid_confirm_window_frames', default_value='3'),
        DeclareLaunchArgument(
            'static_rearm_clear_frames', default_value='5'),
        DeclareLaunchArgument(
            'static_obstacle_speed',
            default_value='7.0',
            description='Static-obstacle approach/avoid/return speed cap.'),
        DeclareLaunchArgument(
            'static_speed_trigger_distance_m', default_value='10.0'),
        DeclareLaunchArgument(
            'static_overtake_trigger_distance_m', default_value='4.5'),
        DeclareLaunchArgument(
            'static_lane_determination_distance_m', default_value='5.0'),
        DeclareLaunchArgument('lidar_rotation_deg', default_value='0.0'),
        DeclareLaunchArgument(
            'cone_entry_fusion_rotation_deg',
            default_value='-125.0',
        ),
        DeclareLaunchArgument(
            'cone_entry_fusion_projection_model',
            default_value='fisheye',
        ),
        DeclareLaunchArgument(
            'cone_entry_use_fusion_gate',
            default_value='true',
            description=(
                'Use camera-LiDAR matches only as an additional CONE entry '
                'gate. Cone driving remains LiDAR-only.'
            ),
        ),
        DeclareLaunchArgument(
            'cone_entry_allow_lidar_fallback',
            default_value='true',
            description=(
                'Allow the established LiDAR entry condition to enter CONE '
                'even when entry fusion is unavailable. When scene inputs '
                'are enabled, fresh qualifying YOLO cones are still required '
                'to prevent an early LiDAR-only false entry.'
            ),
        ),
        DeclareLaunchArgument(
            'cone_entry_min_fused_clusters',
            default_value='2',
        ),
        DeclareLaunchArgument(
            'cone_entry_require_both_sides',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'cone_entry_side_min_abs_y_m',
            default_value='0.10',
        ),
        DeclareLaunchArgument(
            'cone_entry_fusion_timeout_s',
            default_value='0.60',
        ),
        DeclareLaunchArgument(
            'cone_entry_fusion_max_range_m',
            default_value='1.50',
            description='Strict final-entry fusion range.',
        ),
        DeclareLaunchArgument(
            'cone_prearm_enabled',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'cone_bbox_prearm_enabled',
            default_value='true',
            description=(
                'Use qualifying camera cone bboxes to cap lane speed before '
                'the final cone-entry fusion gate.'
            ),
        ),
        DeclareLaunchArgument(
            'cone_prearm_use_fusion',
            default_value='false',
            description=(
                'Also allow camera-LiDAR prearm fusion to activate the '
                'pre-entry speed cap.'
            ),
        ),
        DeclareLaunchArgument(
            'cone_prearm_max_range_m',
            default_value='3.00',
        ),
        DeclareLaunchArgument(
            'cone_prearm_min_bbox_area_px',
            default_value='100.0',
        ),
        DeclareLaunchArgument(
            'cone_prearm_min_bottom_y_px',
            default_value='170.0',
        ),
        DeclareLaunchArgument(
            'cone_bbox_prearm_min_detections',
            default_value='1',
        ),
        DeclareLaunchArgument(
            'cone_bbox_prearm_clear_frames',
            default_value='3',
            description=(
                'Consecutive bbox-missing frames required to release the '
                'camera-only prearm speed cap.'
            ),
        ),
        DeclareLaunchArgument(
            'cone_prearm_min_clusters',
            default_value='1',
        ),
        DeclareLaunchArgument(
            'cone_prearm_window',
            default_value='3',
        ),
        DeclareLaunchArgument(
            'cone_prearm_min_matches',
            default_value='2',
        ),
        DeclareLaunchArgument(
            'cone_prearm_hold_s',
            default_value='0.60',
        ),
        DeclareLaunchArgument(
            'cone_prearm_speed_cap',
            default_value='4.0',
        ),
        DeclareLaunchArgument(
            'cone_prearm_dbscan_eps_m',
            default_value='0.15',
        ),
        DeclareLaunchArgument(
            'cone_prearm_dbscan_min_samples',
            default_value='3',
        ),
        DeclareLaunchArgument(
            'cone_suspect_min_fused_clusters',
            default_value='1',
        ),
        DeclareLaunchArgument(
            'cone_suspect_hold_s',
            default_value='0.70',
        ),
        DeclareLaunchArgument(
            'cone_suspect_speed_cap',
            default_value='0.0',
            description=(
                'Maximum lane speed while partial entry-fusion evidence is '
                'present. Set <= 0 to disable.'
            ),
        ),
        DeclareLaunchArgument(
            'cone_perception_mode',
            default_value='lidar_dbscan',
            choices=['lidar', 'lidar_dbscan'],
            description=(
                'LiDAR cone-driving source. Camera fusion is entry-only.'
            ),
        ),
        DeclareLaunchArgument(
            'lidar_cluster_min_points', default_value='2'),
        DeclareLaunchArgument(
            'lidar_cluster_min_chord_m', default_value='0.02'),
        DeclareLaunchArgument(
            'lidar_cluster_max_radius_m', default_value='0.12'),
        DeclareLaunchArgument(
            'lidar_cluster_max_chord_m', default_value='0.24'),
        DeclareLaunchArgument(
            'lidar_association_distance_m', default_value='0.22'),
        DeclareLaunchArgument('lidar_min_track_hits', default_value='2'),
        DeclareLaunchArgument('lidar_max_track_misses', default_value='2'),
        DeclareLaunchArgument('lidar_min_entry_pairs', default_value='3'),
        DeclareLaunchArgument(
            'lidar_entry_min_clusters',
            default_value='6',
            description=(
                'Minimum structurally accepted LiDAR cones required to enter '
                'CONE mode. Three corridor pairs produce six cones.'
            ),
        ),
        DeclareLaunchArgument(
            'lidar_min_pair_span_m', default_value='0.18'),
        DeclareLaunchArgument(
            'lidar_max_entry_width_spread_m', default_value='0.20'),
        DeclareLaunchArgument(
            'lidar_allow_cone_reentry',
            default_value='false',
            description=(
                'Allow a second cone mission after the first LiDAR cone '
                'corridor has ended.'
            ),
        ),
        DeclareLaunchArgument(
            'lidar_boundary_tolerance_m', default_value='0.18'),
        DeclareLaunchArgument(
            'lidar_min_one_side_points', default_value='2'),
        DeclareLaunchArgument(
            'lidar_two_point_support_confirm_frames', default_value='2'),
        DeclareLaunchArgument(
            'lidar_established_hold_frames', default_value='10'),
        DeclareLaunchArgument(
            'lidar_dbscan_entry_min_clusters',
            default_value='3',
            description=(
                'Minimum DBSCAN angular-bin clusters required before entering '
                'CONE mode.'
            ),
        ),
        DeclareLaunchArgument(
            'lidar_dbscan_allow_cone_reentry',
            default_value='true',
            description=(
                'Allow DBSCAN cone mode to recover after a transient path '
                'loss exits CONE before the course is actually complete.'
            ),
        ),
        DeclareLaunchArgument(
            'lidar_dbscan_min_range_m', default_value='0.18'),
        DeclareLaunchArgument(
            'lidar_dbscan_max_range_m', default_value='1.50'),
        DeclareLaunchArgument('lidar_dbscan_eps_m', default_value='0.04'),
        DeclareLaunchArgument(
            'lidar_dbscan_min_samples', default_value='3'),
        DeclareLaunchArgument(
            'lidar_dbscan_max_cone_diameter_m', default_value='0.30'),
        DeclareLaunchArgument(
            'lidar_dbscan_angle_bin_deg', default_value='0.0'),
        DeclareLaunchArgument(
            'lidar_dbscan_min_cluster_separation_m', default_value='0.15'),
        DeclareLaunchArgument(
            'lidar_dbscan_seed_max_range_m', default_value='0.80'),
        DeclareLaunchArgument(
            'lidar_dbscan_grow_distance_m', default_value='0.50'),
        DeclareLaunchArgument(
            'lidar_dbscan_min_pair_distance_m', default_value='0.45'),
        DeclareLaunchArgument(
            'lidar_dbscan_max_pair_distance_m', default_value='1.30'),
        DeclareLaunchArgument(
            'lidar_dbscan_expected_pair_distance_m', default_value='1.00'),
        DeclareLaunchArgument(
            'lidar_dbscan_max_pair_longitudinal_m', default_value='0.55'),
        DeclareLaunchArgument(
            'lidar_dbscan_midpoint_reference_gate_m', default_value='0.55'),
        DeclareLaunchArgument(
            'lidar_dbscan_path_smoothing_alpha', default_value='0.90'),
        DeclareLaunchArgument(
            'lidar_dbscan_max_path_lateral_jump_m', default_value='0.12'),
        DeclareLaunchArgument(
            'lidar_dbscan_max_path_heading_jump_deg', default_value='30.0'),
        DeclareLaunchArgument(
            'lidar_dbscan_max_path_curvature_per_m', default_value='6.0'),
        DeclareLaunchArgument(
            'lidar_dbscan_path_lost_keep_frames', default_value='4'),
        DeclareLaunchArgument(
            'lidar_dbscan_path_lost_keep_s', default_value='0.45'),
        DeclareLaunchArgument(
            'lidar_dbscan_motion_compensation_enabled',
            default_value='true'),
        DeclareLaunchArgument(
            'lidar_dbscan_prediction_min_forward_m', default_value='0.90'),
        DeclareLaunchArgument(
            'lidar_dbscan_min_path_horizon_m', default_value='0.90'),
        DeclareLaunchArgument(
            'lidar_dbscan_prediction_max_extension_heading_deg',
            default_value='35.0'),
        DeclareLaunchArgument(
            'lidar_dbscan_one_side_measurement_alpha',
            default_value='0.25'),
        DeclareLaunchArgument(
            'lidar_dbscan_one_side_max_lateral_correction_m',
            default_value='0.20'),
        DeclareLaunchArgument(
            'lidar_dbscan_one_side_max_heading_correction_deg',
            default_value='12.0'),
        DeclareLaunchArgument(
            'lidar_dbscan_one_side_max_age_since_paired_s',
            default_value='0.45'),
        DeclareLaunchArgument(
            'lidar_dbscan_virtual_half_width_m', default_value='0.50'),
        DeclareLaunchArgument(
            'lidar_dbscan_rear_axle_offset_m', default_value='0.42'),
        DeclareLaunchArgument(
            'lidar_dbscan_min_lookahead_m', default_value='0.70'),
        DeclareLaunchArgument(
            'lidar_dbscan_max_lookahead_m', default_value='2.50'),
        DeclareLaunchArgument(
            'lidar_dbscan_lookahead_scale', default_value='0.25'),

        DeclareLaunchArgument('lane_straight_speed', default_value='20.0'),
        DeclareLaunchArgument(
            'lane_curve_strong_speed', default_value='10.0'),
        DeclareLaunchArgument(
            'lane_curve_medium_speed', default_value='12.0'),
        DeclareLaunchArgument(
            'lane_curve_mild_speed', default_value='14.0'),
        DeclareLaunchArgument(
            'lane_straight_recovery_speed', default_value='14.0'),
        DeclareLaunchArgument(
            'lane_curve_strong_threshold', default_value='0.60'),
        DeclareLaunchArgument(
            'lane_curve_medium_threshold', default_value='0.35'),
        DeclareLaunchArgument(
            'lane_curve_accel_rate', default_value='6.0'),
        DeclareLaunchArgument(
            'lane_straight_accel_rate', default_value='12.0'),
        DeclareLaunchArgument(
            'lane_curve_decel_rate', default_value='96.0'),
        DeclareLaunchArgument(
            'lane_straight_decel_rate', default_value='36.0'),
        DeclareLaunchArgument(
            'lane_straight_enter_cte_px', default_value='45.0'),
        DeclareLaunchArgument(
            'lane_straight_exit_cte_px', default_value='60.0'),
        DeclareLaunchArgument(
            'lane_straight_enter_heading_deg', default_value='5.0'),
        DeclareLaunchArgument(
            'lane_straight_exit_heading_deg', default_value='8.0'),
        DeclareLaunchArgument(
            'lane_straight_enter_curvature_risk', default_value='0.20'),
        DeclareLaunchArgument(
            'lane_straight_exit_curvature_risk', default_value='0.35'),
        DeclareLaunchArgument(
            'lane_curve_confirmation_frames', default_value='2'),
        DeclareLaunchArgument(
            'lane_straight_confirmation_frames', default_value='2'),
        DeclareLaunchArgument(
            'lane_curvature_filter_window', default_value='3'),
        DeclareLaunchArgument(
            'lane_curvature_scale', default_value='0.50'),
        DeclareLaunchArgument('stanley_control_rate_hz', default_value='20.0'),
        DeclareLaunchArgument(
            'mission_decision_rate_hz', default_value='20.0'),
        DeclareLaunchArgument('stanley_state_timeout_s', default_value='0.80'),
        DeclareLaunchArgument(
            'stanley_hold_last_angle_when_stale',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'stanley_curvature_steering_rate_enabled',
            default_value='false',
            description=(
                'Enable source-time curvature-aware steering slew limits. '
                'False preserves the legacy 15 servo-command per frame limit.'
            ),
        ),
        DeclareLaunchArgument(
            'stanley_steering_rate_straight_per_s', default_value='120.0'),
        DeclareLaunchArgument(
            'stanley_steering_rate_curve_per_s', default_value='300.0'),
        DeclareLaunchArgument(
            'stanley_steering_rate_reversal_per_s', default_value='360.0'),
        DeclareLaunchArgument(
            'stanley_steering_rate_risk_low', default_value='0.15'),
        DeclareLaunchArgument(
            'stanley_steering_rate_risk_high', default_value='0.60'),
        DeclareLaunchArgument(
            'stanley_steering_rate_hold_frames', default_value='4'),
        DeclareLaunchArgument('stanley_image_center_x', default_value='320.0'),
        DeclareLaunchArgument(
            'stanley_gain_schedule_start_speed', default_value='4.0'),
        DeclareLaunchArgument(
            'stanley_gain_schedule_end_speed', default_value='12.0'),
        DeclareLaunchArgument(
            'stanley_k_straight_low_speed', default_value='1.0'),
        DeclareLaunchArgument(
            'stanley_k_straight_high_speed', default_value='0.65'),
        DeclareLaunchArgument(
            'stanley_k_curve_low_speed', default_value='1.20'),
        DeclareLaunchArgument(
            'stanley_k_curve_high_speed', default_value='0.90'),
        DeclareLaunchArgument(
            'stanley_k_obstacle', default_value='1.80'),
        DeclareLaunchArgument(
            'stanley_adaptive_heading_weight_enabled',
            default_value='true'),
        DeclareLaunchArgument(
            'stanley_heading_weight_low_speed', default_value='0.30'),
        DeclareLaunchArgument(
            'stanley_heading_weight_straight_high_speed',
            default_value='0.18'),
        DeclareLaunchArgument(
            'stanley_heading_weight_hough_high_speed',
            default_value='0.14'),
        DeclareLaunchArgument(
            'stanley_heading_weight_curve_high_speed',
            default_value='0.30'),
        DeclareLaunchArgument(
            'stanley_heading_weight_rise_alpha', default_value='0.65'),
        DeclareLaunchArgument(
            'stanley_heading_weight_fall_alpha', default_value='0.35'),
        DeclareLaunchArgument(
            'stanley_accel_rate_per_s', default_value='12.0'),
        DeclareLaunchArgument(
            'stanley_decel_rate_per_s', default_value='36.0'),
        DeclareLaunchArgument(
            'cone_controller',
            default_value='pure_pursuit',
            choices=['pure_pursuit', 'stanley'],
            description='Controller used to track the LiDAR cone path.',
        ),
        DeclareLaunchArgument(
            'cone_min_speed', default_value='4.0',
            description='Minimum cone-course speed command.'),
        DeclareLaunchArgument(
            'cone_max_speed', default_value='4.0',
            description='Maximum cone-course speed command.'),
        DeclareLaunchArgument(
            'cone_entry_speed', default_value='4.0',
            description='Cone entry speed cap during entry_speed_duration_s.'),
        DeclareLaunchArgument(
            'cone_entry_speed_duration_s', default_value='0.60'),
        DeclareLaunchArgument(
            'cone_steering_preview_m', default_value='0.38'),
        DeclareLaunchArgument('cone_speed_preview_m', default_value='0.85'),
        DeclareLaunchArgument('cone_curvature_span_m', default_value='0.14'),
        DeclareLaunchArgument(
            'cone_curvature_feedforward_gain', default_value='1.00'),
        DeclareLaunchArgument('cone_heading_gain', default_value='0.80'),
        DeclareLaunchArgument('cone_cross_track_gain', default_value='0.90'),
        DeclareLaunchArgument(
            'cone_lateral_accel_limit_mps2', default_value='0.30'),
        DeclareLaunchArgument(
            'cone_speed_accel_rate_per_s', default_value='4.0'),
        DeclareLaunchArgument(
            'cone_speed_decel_rate_per_s', default_value='20.0'),
        DeclareLaunchArgument(
            'cone_steering_max_angle_step',
            default_value='12.0',
            description=(
                'Maximum cone steering-command change per fresh path update '
                '(command units). Try 12, 15, then 18 if S reversal is late.'),
        ),
        DeclareLaunchArgument(
            'cone_steering_smoothing_alpha',
            default_value='0.65',
            description='Cone steering low-pass new-sample weight [0, 1].'),
        DeclareLaunchArgument(
            'cone_confidence_min',
            default_value='0.35',
            description=(
                'Path confidence at/below which cone speed is capped to '
                'cone_min_speed.'),
        ),
        DeclareLaunchArgument(
            'cone_confidence_full',
            default_value='1.0',
            description=(
                'Path confidence at which the confidence speed cap reaches '
                'cone_max_speed.'),
        ),
        DeclareLaunchArgument('cone_min_lookahead_m', default_value='0.30'),
        DeclareLaunchArgument('cone_max_lookahead_m', default_value='0.60'),
        DeclareLaunchArgument('cone_lookahead_scale', default_value='0.30'),
        DeclareLaunchArgument(
            'cone_stanley_control_x_m', default_value='0.33'),
        DeclareLaunchArgument(
            'cone_stanley_heading_window_m', default_value='0.30'),
        DeclareLaunchArgument(
            'cone_stanley_cross_track_gain', default_value='1.20'),
        DeclareLaunchArgument(
            'cone_stanley_heading_gain', default_value='1.00'),
        DeclareLaunchArgument(
            'cone_stanley_softening_speed_mps', default_value='0.50'),
        DeclareLaunchArgument(
            'cone_stanley_max_angle_step', default_value='15.0'),
        DeclareLaunchArgument(
            'cone_stanley_smoothing_alpha', default_value='0.65'),
        DeclareLaunchArgument(
            'cone_stanley_path_timeout_s', default_value='0.50'),

        DeclareLaunchArgument('max_range_m', default_value='1.80'),
        DeclareLaunchArgument('x_max_m', default_value='1.80'),
        DeclareLaunchArgument('y_abs_max_m', default_value='0.80'),
        DeclareLaunchArgument('gap_min_width_m', default_value='0.50'),
        DeclareLaunchArgument('gap_max_width_m', default_value='1.00'),
        DeclareLaunchArgument('gap_expected_width_m', default_value='0.80'),
        DeclareLaunchArgument('cone_spacing_m', default_value='0.30'),
        DeclareLaunchArgument(
            'cone_one_side_half_width_m', default_value='0.40'),
        DeclareLaunchArgument(
            'cone_pair_min_normal_alignment', default_value='0.45'),
        DeclareLaunchArgument(
            'cone_pair_max_longitudinal_m', default_value='0.45'),
        DeclareLaunchArgument(
            'cone_midpoint_reference_gate_m', default_value='0.45'),
        DeclareLaunchArgument(
            'cone_min_midpoints_for_path', default_value='2'),
        DeclareLaunchArgument('cone_min_path_span_m', default_value='0.35'),
        DeclareLaunchArgument(
            'cone_max_path_lateral_jump_m',
            default_value='0.45',
        ),
        DeclareLaunchArgument(
            'cone_path_lost_keep_frames', default_value='3'),
        DeclareLaunchArgument(
            'cone_path_smoothing_alpha', default_value='0.75'),
        DeclareLaunchArgument('bbox_padding_x_px', default_value='20.0'),
        DeclareLaunchArgument('bbox_padding_y_px', default_value='30.0'),
        DeclareLaunchArgument('cone_entry_min_detections', default_value='2'),
        DeclareLaunchArgument('cone_entry_min_path_points', default_value='5'),
        DeclareLaunchArgument('cone_enter_confirm_cycles', default_value='3'),
        DeclareLaunchArgument(
            'cone_entry_min_bottom_y_px',
            default_value='220.0',
        ),
        DeclareLaunchArgument('cone_exit_hold_s', default_value='1.0'),
        DeclareLaunchArgument(
            'cone_min_mode_duration_s', default_value='3.0'),

        OpaqueFunction(function=validate_and_log_configuration),

        SetParameter(
            name='use_sim_time',
            value=ParameterValue(use_sim_time, value_type=bool),
        ),
        LogInfo(
            condition=IfCondition(PythonExpression([
                "'", enable_drive_control, "'.lower() == 'true' and '",
                enable_lane_pipeline, "'.lower() != 'true'",
            ])),
            msg=(
                'enable_drive_control=true was ignored because '
                'enable_lane_pipeline=false.'
            ),
        ),

        Node(
            package='vesc_driver',
            executable='vesc_driver_node',
            name='vesc_driver_node',
            output='screen',
            condition=IfCondition(start_motor_driver),
            parameters=[{
                'port': motor_port,
                'auto_device_symlink': motor_auto_device_symlink,
                'auto_device_match': motor_auto_device_match,
                'handshake_timeout_s': ParameterValue(
                    vesc_handshake_timeout_s, value_type=float),
                'reconnect_interval_s': ParameterValue(
                    vesc_reconnect_interval_s, value_type=float),
                'telemetry_timeout_s': ParameterValue(
                    vesc_telemetry_timeout_s, value_type=float),
                'poll_imu': False,
                'duty_cycle_min': -0.95,
                'duty_cycle_max': 0.95,
                'current_min': -5.0,
                'current_max': 15.0,
                'brake_min': 0.0,
                'brake_max': 0.0,
                'speed_min': -10000.0,
                'speed_max': 10000.0,
                'position_min': 0.0,
                'position_max': 0.0,
                'servo_min': 0.15,
                'servo_max': 0.85,
            }],
        ),
        Node(
            package='xycar_motor_native',
            executable='motor_command_adapter',
            name='xycar_motor_adapter',
            output='screen',
            condition=IfCondition(start_motor_driver),
            parameters=[{
                'input_topic': motor_topic,
                'control_mode': 'duty_speed',
            }],
        ),

        include_package_launch(
            'xycar_cam',
            'xycar_cam.launch.py',
            start_camera,
            {
                'publish_camera_timing': publish_camera_timing,
                'camera_timing_topic': camera_timing_topic,
            },
        ),
        Node(
            package='cam',
            executable='camera_timing_probe',
            name='camera_timing_probe',
            output='screen',
            condition=IfCondition(PythonExpression([
                "'", start_camera, "'.lower() == 'true' and '",
                publish_camera_timing, "'.lower() == 'true'",
            ])),
            parameters=[{
                'driver_timing_topic': camera_timing_topic,
                'complete_timing_topic': '/camera_timing_complete',
            }],
        ),
        include_package_launch(
            'xycar_lidar', 'xycar_lidar.launch.py', lidar_driver_enabled),
        include_package_launch(
            'xycar_ultrasonic',
            'xycar_ultrasonic.launch.py',
            ultrasonic_stack_enabled,
        ),

        Node(
            package='cam',
            executable='frame_router',
            name='frame_router',
            output='screen',
            condition=IfCondition(any_perception_pipeline_enabled),
            parameters=[{
                'input_image_topic': input_image_topic,
                'camera_qos_reliability': camera_qos_reliability,
                'lane_image_topic': lane_image_topic,
                'lane_image_width': lane_image_width,
                'lane_image_height': lane_image_height,
                'lane_publish_rate_hz': lane_max_rate_hz,
                'lane_event_driven': ParameterValue(
                    frame_router_event_driven, value_type=bool),
                'scene_image_topic': scene_image_topic,
                'scene_image_width': scene_image_width,
                'scene_image_height': scene_image_height,
                'scene_publish_rate_hz': scene_max_rate_hz,
                'lane_output_qos_reliability':
                    lane_image_qos_reliability,
                'scene_output_qos_reliability':
                    scene_image_qos_reliability,
                'enable_lane_output': ParameterValue(
                    enable_lane_pipeline,
                    value_type=bool,
                ),
                'enable_scene_output': ParameterValue(
                    enable_scene_pipeline,
                    value_type=bool,
                ),
                'publish_performance_stats': publish_performance_stats,
                'performance_log_period_s': performance_log_period_s,
                'publish_pipeline_timing': ParameterValue(
                    publish_pipeline_timing, value_type=bool),
                'pipeline_timing_topic': pipeline_timing_topic,
            }],
        ),
        Node(
            package='cam',
            executable='yolo_node',
            name='lane_yolo_node',
            output='screen',
            condition=IfCondition(lane_yolo_enabled),
            prefix=[
                'taskset -c ', lane_cpu_affinity,
                ' nice -n ', lane_process_nice,
            ],
            additional_env={
                'OMP_NUM_THREADS': lane_torch_threads,
                'MKL_NUM_THREADS': lane_torch_threads,
                'OPENBLAS_NUM_THREADS': '1',
                'OMP_DYNAMIC': 'FALSE',
                'TORCHINDUCTOR_COMPILE_THREADS': '2',
            },
            parameters=[{
                'image_topic_name': lane_image_topic,
                'image_qos_reliability': lane_image_qos_reliability,
                'detections_topic': '/lane_yolo/detections',
                'source_image_topic': '/lane_yolo/source_image',
                'annotated_image_topic': '/lane_yolo/annotated_image',
                'model_filename': lane_model_filename,
                'inference_class_names': lane_class_names,
                'max_inference_rate_hz': lane_yolo_max_rate_hz,
                'inference_image_size': lane_inference_image_size,
                'torch_num_threads': lane_torch_threads,
                'torch_num_interop_threads': lane_torch_interop_threads,
                'opencv_num_threads': lane_opencv_threads,
                'torch_compile_mode': lane_torch_compile_mode,
                'compile_input_width': lane_image_width,
                'compile_input_height': lane_image_height,
                'compile_warmup_iterations':
                    lane_compile_warmup_iterations,
                'inference_device': ParameterValue(
                    lane_yolo_device, value_type=str),
                'use_fp16': ParameterValue(
                    lane_yolo_fp16, value_type=bool),
                'runtime_report_path': lane_yolo_runtime_report_path,
                'publish_source_image': False,
                'publish_annotated_image': show_lane_yolo_view,
                'annotated_max_rate_hz': lane_annotated_max_rate_hz,
                'show_visualization': show_lane_yolo_view,
                'show_labels': True,
                'show_conf': True,
                'enable_checkerboard': False,
                'publish_performance_stats': publish_performance_stats,
                'performance_log_period_s': performance_log_period_s,
                'publish_pipeline_timing': ParameterValue(
                    publish_pipeline_timing, value_type=bool),
                'pipeline_timing_topic': pipeline_timing_topic,
            }],
        ),
        Node(
            package='cam',
            executable='resource_profiler',
            name='resource_profiler',
            output='screen',
            condition=IfCondition(publish_resource_stats),
            parameters=[{
                'output_path': resource_stats_output_path,
                'sample_rate_hz': ParameterValue(
                    resource_stats_rate_hz, value_type=float),
                'target_process_pattern': resource_stats_process_pattern,
                'gpu_index': ParameterValue(
                    resource_stats_gpu_index, value_type=int),
                'profile_target_threads': ParameterValue(
                    resource_stats_profile_threads, value_type=bool),
            }],
        ),
        Node(
            package='cam',
            executable='yolo_node',
            name='scene_yolo_node',
            output='screen',
            condition=IfCondition(scene_yolo_enabled),
            prefix=[
                'taskset -c ', scene_cpu_affinity,
                ' nice -n ', scene_process_nice,
            ],
            additional_env={
                'OMP_NUM_THREADS': scene_torch_threads,
                'MKL_NUM_THREADS': scene_torch_threads,
                'OPENBLAS_NUM_THREADS': '1',
                'OMP_DYNAMIC': 'FALSE',
                'TORCHINDUCTOR_COMPILE_THREADS': '2',
            },
            parameters=[{
                'image_topic_name': scene_image_topic,
                'image_qos_reliability': scene_image_qos_reliability,
                'detections_topic': '/scene_yolo/detections',
                'source_image_topic': '/scene_yolo/source_image',
                'annotated_image_topic': '/scene_yolo/annotated_image',
                'model_filename': scene_model_filename,
                'inference_class_names': scene_class_names,
                'class_name_aliases': scene_class_name_aliases,
                'enable_checkerboard': scene_enable_checkerboard,
                'general_conf_threshold': scene_general_conf_threshold,
                'class_1_conf_threshold':
                    scene_obstacle_conf_threshold,
                'cone_conf_threshold': cone_conf_threshold,
                'max_inference_rate_hz': scene_yolo_max_rate_hz,
                'inference_image_size': scene_inference_image_size,
                'torch_num_threads': scene_torch_threads,
                'torch_num_interop_threads':
                    scene_torch_interop_threads,
                'opencv_num_threads': scene_opencv_threads,
                'torch_compile_mode': scene_torch_compile_mode,
                'compile_input_width': scene_image_width,
                'compile_input_height': scene_image_height,
                'compile_warmup_iterations':
                    scene_compile_warmup_iterations,
                'inference_device': ParameterValue(
                    scene_yolo_device, value_type=str),
                'use_fp16': ParameterValue(
                    scene_yolo_fp16, value_type=bool),
                'runtime_report_path': scene_yolo_runtime_report_path,
                'publish_source_image': False,
                'publish_annotated_image': show_scene_yolo_view,
                'annotated_max_rate_hz': scene_annotated_max_rate_hz,
                'show_visualization': show_scene_yolo_view,
                'show_labels': True,
                'show_conf': True,
                'publish_performance_stats': publish_performance_stats,
                'performance_log_period_s': performance_log_period_s,
            }],
        ),
        Node(
            package='cam',
            executable='centerlane_tracer',
            name='centerlane_tracer',
            output='screen',
            condition=IfCondition(lane_stack_enabled),
            parameters=[{
                'debug_view': show_lane_processing_view,
                'verbose_sort_debug': False,
                'image_topic': lane_image_topic,
                'lane_detections_topic': '/lane_yolo/detections',
                'scene_detections_topic': '/scene_yolo/detections',
                'center_curve_topic': center_curve_output_topic,
                'image_qos_reliability': lane_image_qos_reliability,
                'base_input_width': lane_base_input_width,
                'base_input_height': lane_base_input_height,
                'height_threshold_px': lane_center_height_threshold_px,
                'adaptive_block_size_px':
                    lane_center_adaptive_block_size_px,
                'hough_threshold': lane_center_hough_threshold,
                'hough_min_line_length_px':
                    lane_center_hough_min_line_length_px,
                'hough_max_line_gap_px':
                    lane_center_hough_max_line_gap_px,
                'similarity_threshold_px':
                    lane_center_similarity_threshold_px,
                'rmse_threshold_px': lane_center_rmse_threshold_px,
                'bottom_box_height_threshold_px':
                    lane_center_bottom_box_height_threshold_px,
                'bottom_point_height_diff_threshold_px':
                    lane_center_bottom_point_height_diff_threshold_px,
                'min_bbox_width_px': lane_center_min_bbox_width_px,
                'min_bbox_height_px': lane_center_min_bbox_height_px,
                'x_distance_threshold_px':
                    lane_center_x_distance_threshold_px,
                'point_merge_distance_px':
                    lane_center_point_merge_distance_px,
                'spline_smoothing_px2':
                    lane_center_spline_smoothing_px2,
                'enable_scene_detection_cache': ParameterValue(
                    enable_scene_pipeline,
                    value_type=bool,
                ),
                'scene_detection_max_age_s': scene_detection_max_age_s,
                'scene_detection_source_width': scene_image_width,
                'scene_detection_source_height': scene_image_height,
                'publish_performance_stats': publish_performance_stats,
                'performance_log_period_s': performance_log_period_s,
                'publish_pipeline_timing': ParameterValue(
                    publish_pipeline_timing, value_type=bool),
                'pipeline_timing_topic': pipeline_timing_topic,
                'sync_slop': sync_slop,
                'sync_queue_size': sync_queue_size,
            }],
        ),
        Node(
            package='cam',
            executable='shortcut_left_turn_node',
            name='shortcut_left_turn_node',
            output='screen',
            condition=IfCondition(shortcut_left_turn_enabled),
            parameters=[{
                'image_topic': lane_image_topic,
                'detections_topic': '/lane_yolo/detections',
                'base_curve_topic': '/center_curve/base',
                'output_curve_topic': '/center_curve',
                'traffic_raw_state_topic': '/traffic_light_raw_state',
                'lane_override_topic': '/lane_override_cmd',
                'image_qos_reliability': lane_image_qos_reliability,
                'auto_trigger_after_s': ParameterValue(
                    shortcut_auto_trigger_after_s, value_type=float),
                'auto_trigger_state': shortcut_auto_trigger_state,
                'prepare_speed_limit': ParameterValue(
                    shortcut_prepare_speed_limit, value_type=float),
                'turn_speed_limit': ParameterValue(
                    shortcut_turn_speed_limit, value_type=float),
                'cruise_speed_limit': ParameterValue(
                    shortcut_cruise_speed_limit, value_type=float),
                'left_signal_window': ParameterValue(
                    shortcut_left_signal_window, value_type=int),
                'left_signal_min_matches': ParameterValue(
                    shortcut_left_signal_min_matches, value_type=int),
                'fallback_commit_s': ParameterValue(
                    shortcut_fallback_commit_s, value_type=float),
                'entry_max_heading_deg': ParameterValue(
                    shortcut_entry_max_heading_deg, value_type=float),
                'min_cruise_duration_s': ParameterValue(
                    shortcut_min_cruise_duration_s, value_type=float),
                'crossline_window': ParameterValue(
                    shortcut_crossline_window, value_type=int),
                'crossline_min_matches': ParameterValue(
                    shortcut_crossline_min_matches, value_type=int),
                'exit_min_turn_duration_s': ParameterValue(
                    shortcut_exit_min_turn_duration_s, value_type=float),
                'exit_alignment_window': ParameterValue(
                    shortcut_exit_alignment_window, value_type=int),
                'exit_alignment_min_matches': ParameterValue(
                    shortcut_exit_alignment_min_matches, value_type=int),
                'exit_handoff_duration_s': ParameterValue(
                    shortcut_exit_handoff_duration_s, value_type=float),
                'exit_max_heading_deg': ParameterValue(
                    shortcut_exit_max_heading_deg, value_type=float),
            }],
        ),
        Node(
            package='cam',
            executable='traffic_light_node',
            name='traffic_light_node',
            output='screen',
            condition=IfCondition(enable_scene_pipeline),
            parameters=[{
                'image_topic': scene_image_topic,
                'detection_topic': '/scene_yolo/detections',
                'image_qos_reliability': scene_image_qos_reliability,
                'sync_slop': sync_slop,
                'sync_queue_size': sync_queue_size,
                'direct_state_min_confidence':
                    traffic_direct_state_min_confidence,
                'direct_red_min_confidence':
                    traffic_direct_red_min_confidence,
                'direct_green_min_confidence':
                    traffic_direct_green_min_confidence,
                'direct_left_min_confidence':
                    traffic_direct_left_min_confidence,
                'stable_frames': traffic_stable_frames,
                'left_is_green': ParameterValue(
                    traffic_left_is_green,
                    value_type=bool,
                ),
                'red_latch_until_green': ParameterValue(
                    traffic_red_latch_until_green,
                    value_type=bool,
                ),
                'red_latch_confirm_frames':
                    traffic_red_latch_confirm_frames,
                'debug_view': show_traffic_light_view,
            }],
        ),
        Node(
            package='cam',
            executable='lane_detector',
            name='lane_detector',
            output='screen',
            condition=IfCondition(lane_stack_enabled),
            parameters=[{
                'start_service_name': '/start_lane_detection',
                'image_topic': lane_image_topic,
                'lane_detections_topic': '/lane_yolo/detections',
                'scene_detections_topic': '/scene_yolo/detections',
                'center_curve_topic': '/center_curve',
                'shortcut_path_active_topic':
                    '/shortcut_left_turn/path_active',
                'state_topic': '/xycar_state',
                'stamped_state_topic': '/xycar_state_stamped',
                'lane_control_state_topic':
                    '/lane_control_state_stamped',
                'lane_control_state_v2_topic':
                    '/lane_control_state_v2',
                'lane_control_state_v3_topic':
                    '/lane_control_state_v3',
                'publish_legacy_state': True,
                'publish_stamped_state': True,
                'publish_lane_control_state': True,
                'publish_lane_control_state_v2': True,
                'publish_lane_control_state_v3': True,
                'image_qos_reliability': lane_image_qos_reliability,
                'base_input_width': lane_base_input_width,
                'base_input_height': lane_base_input_height,
                'canny_blur_kernel_px': lane_canny_blur_kernel_px,
                'object_grid_cell_size_px':
                    lane_object_grid_cell_size_px,
                'curve_heading_recovery_enabled': ParameterValue(
                    lane_curve_heading_recovery_enabled,
                    value_type=bool,
                ),
                'curve_heading_disagreement_deg':
                    lane_curve_heading_disagreement_deg,
                'curve_heading_recovery_weight':
                    lane_curve_heading_recovery_weight,
                'curve_heading_confirm_frames':
                    lane_curve_heading_confirm_frames,
                'curve_heading_curve_max_fit_error_ratio':
                    lane_curve_heading_curve_max_fit_error_ratio,
                'curve_heading_extrapolation_margin_ratio':
                    lane_curve_heading_extrapolation_margin_ratio,
                'adaptive_heading_enabled': ParameterValue(
                    lane_adaptive_heading_enabled,
                    value_type=bool,
                ),
                'heading_preview_min_ratio':
                    lane_heading_preview_min_ratio,
                'heading_preview_max_ratio':
                    lane_heading_preview_max_ratio,
                'heading_curvature_scale':
                    lane_heading_curvature_scale,
                'heading_curvature_deadzone':
                    lane_heading_curvature_deadzone,
                'heading_filter_alpha_straight':
                    lane_heading_filter_alpha_straight,
                'heading_filter_alpha_curve':
                    lane_heading_filter_alpha_curve,
                'heading_filter_alpha_fallback':
                    lane_heading_filter_alpha_fallback,
                'heading_curve_hold_frames':
                    lane_heading_curve_hold_frames,
                'heading_max_step_deg': lane_heading_max_step_deg,
                'lateral_innovation_straight_px':
                    lane_lateral_innovation_straight_px,
                'lateral_innovation_curve_px':
                    lane_lateral_innovation_curve_px,
                'lateral_max_rate_straight_px_s':
                    lane_lateral_max_rate_straight_px_s,
                'lateral_lane_half_width_px':
                    lane_lateral_lane_half_width_px,
                'lateral_outer_pair_tolerance_px':
                    lane_lateral_outer_pair_tolerance_px,
                'lateral_outer_pair_hold_frames':
                    lane_lateral_outer_pair_hold_frames,
                'straight_center_agreement_threshold_px':
                    lane_straight_center_agreement_threshold_px,
                'straight_center_correction_weight':
                    lane_straight_center_correction_weight,
                'straight_center_max_jump_px':
                    lane_straight_center_max_jump_px,
                'straight_center_jump_confirm_frames':
                    lane_straight_center_jump_confirm_frames,
                'obstacle_recovery_duration_s':
                    lane_obstacle_recovery_duration_s,
                'curve_center_agreement_threshold_px':
                    lane_curve_center_agreement_threshold_px,
                'curve_center_correction_weight':
                    lane_curve_center_correction_weight,
                'curve_center_hold_frames':
                    lane_curve_center_hold_frames,
                'enable_scene_detection_cache': ParameterValue(
                    enable_scene_pipeline,
                    value_type=bool,
                ),
                'scene_detection_max_age_s': scene_detection_max_age_s,
                'scene_detection_source_width': scene_image_width,
                'scene_detection_source_height': scene_image_height,
                'publish_performance_stats': publish_performance_stats,
                'performance_log_period_s': performance_log_period_s,
                'publish_pipeline_timing': ParameterValue(
                    publish_pipeline_timing, value_type=bool),
                'pipeline_timing_topic': pipeline_timing_topic,
                'sync_slop': sync_slop,
                'sync_queue_size': sync_queue_size,
                'show_debug_windows': show_lane_processing_view,
            }],
        ),
        Node(
            package='cam',
            executable='integrated_stanley_controller',
            name='integrated_stanley_controller',
            output='screen',
            condition=IfCondition(lane_control_enabled),
            parameters=[{
                'start_service_name': '/start_stanley_controller',
                'state_topic': '/xycar_state',
                'stamped_state_topic': '/xycar_state_stamped',
                'lane_control_state_topic':
                    '/lane_control_state_stamped',
                'lane_control_state_v2_topic':
                    '/lane_control_state_v2',
                'lane_control_state_v3_topic':
                    '/lane_control_state_v3',
                'use_lane_control_state_v3': True,
                'use_lane_control_state_v2': True,
                'use_lane_control_state': True,
                'use_stamped_state': True,
                'motor_topic': '/lane_motor_cmd',
                'stamped_motor_topic': '/lane_motor_cmd_stamped',
                'speed_straight': lane_straight_speed,
                'curve_strong_speed': lane_curve_strong_speed,
                'curve_medium_speed': lane_curve_medium_speed,
                'curve_mild_speed': lane_curve_mild_speed,
                'straight_recovery_speed':
                    lane_straight_recovery_speed,
                'curve_strong_threshold': lane_curve_strong_threshold,
                'curve_medium_threshold': lane_curve_medium_threshold,
                'curve_accel_rate_per_s': lane_curve_accel_rate,
                'straight_accel_rate_per_s': lane_straight_accel_rate,
                'curve_decel_rate_per_s': lane_curve_decel_rate,
                'straight_decel_rate_per_s': lane_straight_decel_rate,
                'straight_enter_cte_px':
                    lane_straight_enter_cte_px,
                'straight_exit_cte_px':
                    lane_straight_exit_cte_px,
                'straight_enter_heading_deg':
                    lane_straight_enter_heading_deg,
                'straight_exit_heading_deg':
                    lane_straight_exit_heading_deg,
                'straight_enter_curvature_risk':
                    lane_straight_enter_curvature_risk,
                'straight_exit_curvature_risk':
                    lane_straight_exit_curvature_risk,
                'curve_confirm_frames':
                    lane_curve_confirmation_frames,
                'straight_confirm_frames':
                    lane_straight_confirmation_frames,
                'curvature_filter_window':
                    lane_curvature_filter_window,
                'curvature_scale': lane_curvature_scale,
                'control_rate_hz': stanley_control_rate_hz,
                'event_driven_control': ParameterValue(
                    stanley_event_driven, value_type=bool),
                'state_timeout_s': stanley_state_timeout_s,
                'hold_last_angle_when_stale':
                    stanley_hold_last_angle_when_stale,
                'curvature_steering_rate_enabled': ParameterValue(
                    stanley_curvature_steering_rate_enabled,
                    value_type=bool,
                ),
                'steering_rate_straight_per_s':
                    stanley_steering_rate_straight_per_s,
                'steering_rate_curve_per_s':
                    stanley_steering_rate_curve_per_s,
                'steering_rate_reversal_per_s':
                    stanley_steering_rate_reversal_per_s,
                'steering_rate_risk_low':
                    stanley_steering_rate_risk_low,
                'steering_rate_risk_high':
                    stanley_steering_rate_risk_high,
                'steering_rate_hold_frames':
                    stanley_steering_rate_hold_frames,
                'image_center_x': stanley_image_center_x,
                'gain_schedule_start_speed':
                    stanley_gain_schedule_start_speed,
                'gain_schedule_end_speed':
                    stanley_gain_schedule_end_speed,
                'cross_track_gain_straight_low_speed':
                    stanley_k_straight_low_speed,
                'cross_track_gain_straight_high_speed':
                    stanley_k_straight_high_speed,
                'cross_track_gain_curve_low_speed':
                    stanley_k_curve_low_speed,
                'cross_track_gain_curve_high_speed':
                    stanley_k_curve_high_speed,
                'cross_track_gain_obstacle': stanley_k_obstacle,
                'adaptive_heading_weight_enabled': ParameterValue(
                    stanley_adaptive_heading_weight_enabled,
                    value_type=bool,
                ),
                'heading_weight_low_speed':
                    stanley_heading_weight_low_speed,
                'heading_weight_straight_high_speed':
                    stanley_heading_weight_straight_high_speed,
                'heading_weight_hough_high_speed':
                    stanley_heading_weight_hough_high_speed,
                'heading_weight_curve_high_speed':
                    stanley_heading_weight_curve_high_speed,
                'heading_weight_rise_alpha':
                    stanley_heading_weight_rise_alpha,
                'heading_weight_fall_alpha':
                    stanley_heading_weight_fall_alpha,
                'accel_rate_per_s': stanley_accel_rate_per_s,
                'decel_rate_per_s': stanley_decel_rate_per_s,
                'nominal_source_rate_hz': lane_max_rate_hz,
                'future_stamp_tolerance_s': future_stamp_tolerance_s,
                'enable_scene_inputs': ParameterValue(
                    enable_scene_pipeline,
                    value_type=bool,
                ),
                'respect_traffic_light': ParameterValue(
                    enable_scene_pipeline,
                    value_type=bool,
                ),
                'shortcut_speed_limit_topic':
                    '/shortcut_left_turn/speed_limit',
                'obstacle_speed_limit_topic': '/obstacle_speed_limit',
                'shortcut_path_active_topic':
                    '/shortcut_left_turn/path_active',
                'shortcut_steering_limit': ParameterValue(
                    shortcut_steering_limit, value_type=float),
                'verbose_status': False,
                'publish_pipeline_timing': ParameterValue(
                    publish_pipeline_timing, value_type=bool),
                'pipeline_timing_topic': pipeline_timing_topic,
            }],
        ),
        Node(
            package='cam',
            executable='ultra_node',
            name='ultra_node',
            output='screen',
            condition=IfCondition(ultrasonic_stack_enabled),
            parameters=[{
                'raw_timeout_s': overtake_sensor_timeout_s,
                'publish_rate_hz': 10.0,
            }],
        ),
        Node(
            package='cam',
            executable='target_lane_planner',
            name='target_lane_planner',
            output='screen',
            condition=IfCondition(target_lane_planner_enabled),
            parameters=[{
                'image_topic': scene_image_topic,
                'detections_topic': '/scene_yolo/detections',
                'curve_topic': '/center_curve',
                'road_curve_topic': center_curve_output_topic,
                'road_curve_max_time_delta_s':
                    road_curve_max_time_delta_s,
                'image_qos_reliability': scene_image_qos_reliability,
                'curve_source_width': lane_image_width,
                'curve_source_height': lane_image_height,
                'sync_slop': sync_slop,
                'sync_queue_size': sync_queue_size,
                'sensor_timeout_s': overtake_sensor_timeout_s,
                'obstacle_lidar_min_range_m':
                    obstacle_lidar_min_range_m,
                'obstacle_lidar_max_range_m':
                    obstacle_lidar_max_range_m,
                'obstacle_lidar_cluster_min_points':
                    obstacle_lidar_cluster_min_points,
                'obstacle_lidar_cluster_base_gap_m':
                    obstacle_lidar_cluster_base_gap_m,
                'obstacle_lidar_cluster_range_gap_scale':
                    obstacle_lidar_cluster_range_gap_scale,
                'obstacle_lidar_cluster_max_width_m':
                    obstacle_lidar_cluster_max_width_m,
                'obstacle_lidar_distance_percentile':
                    obstacle_lidar_distance_percentile,
                'obstacle_lidar_association_padding_px':
                    obstacle_lidar_association_padding_px,
                'obstacle_lidar_association_max_above_px':
                    obstacle_lidar_association_max_above_px,
                'obstacle_lidar_camera_max_delta_s':
                    obstacle_lidar_camera_max_delta_s,
                'obstacle_lidar_tracking_enabled': ParameterValue(
                    obstacle_lidar_tracking_enabled,
                    value_type=bool,
                ),
                'obstacle_lidar_track_max_age_s':
                    obstacle_lidar_track_max_age_s,
                'obstacle_lidar_track_max_misses':
                    obstacle_lidar_track_max_misses,
                'obstacle_lidar_track_max_distance_jump_m':
                    obstacle_lidar_track_max_distance_jump_m,
                'obstacle_lidar_track_max_range_rate_mps':
                    obstacle_lidar_track_max_range_rate_mps,
                'obstacle_lidar_track_max_total_distance_jump_m':
                    obstacle_lidar_track_max_total_distance_jump_m,
                'obstacle_lidar_track_max_offset_jump_px':
                    obstacle_lidar_track_max_offset_jump_px,
                'obstacle_lidar_track_max_offset_rate_px_s':
                    obstacle_lidar_track_max_offset_rate_px_s,
                'obstacle_lidar_track_bbox_iou_min':
                    obstacle_lidar_track_bbox_iou_min,
                'obstacle_lidar_track_bbox_center_gate_px':
                    obstacle_lidar_track_bbox_center_gate_px,
                'obstacle_lidar_near_cluster_distance_m':
                    obstacle_lidar_near_cluster_distance_m,
                'obstacle_lidar_near_cluster_min_bbox_height_ratio':
                    obstacle_lidar_near_cluster_min_bbox_height_ratio,
                'obstacle_lidar_near_cluster_min_bbox_bottom_ratio':
                    obstacle_lidar_near_cluster_min_bbox_bottom_ratio,
                'obstacle_camera_fallback_speed_enabled': ParameterValue(
                    obstacle_camera_fallback_speed_enabled,
                    value_type=bool,
                ),
                'obstacle_camera_fallback_min_bottom_ratio':
                    obstacle_camera_fallback_min_bottom_ratio,
                'obstacle_allow_avoid_on_curve': ParameterValue(
                    obstacle_allow_avoid_on_curve,
                    value_type=bool,
                ),
                'dynamic_event_duration_s': dynamic_event_duration_s,
                'dynamic_speed_confirm_min_matches':
                    dynamic_speed_confirm_min_matches,
                'dynamic_speed_confirm_window_frames':
                    dynamic_speed_confirm_window_frames,
                'dynamic_avoid_confirm_min_matches':
                    dynamic_avoid_confirm_min_matches,
                'dynamic_avoid_confirm_window_frames':
                    dynamic_avoid_confirm_window_frames,
                'dynamic_rearm_clear_frames':
                    dynamic_rearm_clear_frames,
                'dynamic_obstacle_speed': dynamic_obstacle_speed,
                'dynamic_speed_trigger_distance_m':
                    dynamic_speed_trigger_distance_m,
                'dynamic_overtake_trigger_distance_m':
                    dynamic_overtake_trigger_distance_m,
                'dynamic_lane_determination_distance_m':
                    dynamic_lane_determination_distance_m,
                'static_event_duration_s': static_event_duration_s,
                'static_speed_confirm_min_matches':
                    static_speed_confirm_min_matches,
                'static_speed_confirm_window_frames':
                    static_speed_confirm_window_frames,
                'static_avoid_confirm_min_matches':
                    static_avoid_confirm_min_matches,
                'static_avoid_confirm_window_frames':
                    static_avoid_confirm_window_frames,
                'static_rearm_clear_frames':
                    static_rearm_clear_frames,
                'static_obstacle_speed': static_obstacle_speed,
                'static_speed_trigger_distance_m':
                    static_speed_trigger_distance_m,
                'static_overtake_trigger_distance_m':
                    static_overtake_trigger_distance_m,
                'static_lane_determination_distance_m':
                    static_lane_determination_distance_m,
                'obstacle_speed_limit_topic': '/obstacle_speed_limit',
                'show_visualization': show_overtake_view,
                'mission_mode_topic': '/mission_mode',
                'shortcut_state_topic': '/shortcut_left_turn/state',
            }],
        ),

        Node(
            package='mission_cone_drive',
            executable='scan_rotator',
            name='scan_rotator_node',
            output='screen',
            condition=IfCondition(cone_drive_enabled),
            parameters=[{
                'input_topic': '/scan',
                'output_topic': '/scan_rotated',
                'rotation_deg': lidar_rotation_deg,
            }],
        ),
        Node(
            package='mission_cone_drive',
            executable='cone_entry_fusion_node',
            name='cone_entry_fusion_node',
            output='screen',
            condition=IfCondition(PythonExpression([
                "'", start_lane_stack, "'.lower() == 'true' and '",
                enable_lane_pipeline, "'.lower() == 'true' and '",
                enable_scene_pipeline, "'.lower() == 'true' and '",
                enable_drive_control, "'.lower() == 'true' and '",
                start_yolo, "'.lower() == 'true' and '",
                cone_entry_use_fusion_gate, "'.lower() == 'true' and '",
                cone_perception_mode, "' == 'lidar_dbscan'",
            ])),
            parameters=[{
                'raw_clusters_topic': '/clusters_raw',
                'prearm_raw_clusters_topic': '/prearm_clusters_raw',
                'detections_topic': '/scene_yolo/detections',
                'output_clusters_topic': '/entry_cone_clusters',
                'prearm_output_clusters_topic':
                    '/cone_prearm_clusters',
                'status_topic': '/cone_entry_fusion_status',
                'cone_conf_threshold': cone_conf_threshold,
                'min_bbox_area_px': 180.0,
                'min_bottom_y_px': cone_entry_min_bottom_y_px,
                'entry_max_range_m': cone_entry_fusion_max_range_m,
                'prearm_min_bbox_area_px':
                    cone_prearm_min_bbox_area_px,
                'prearm_min_bottom_y_px':
                    cone_prearm_min_bottom_y_px,
                'prearm_max_range_m': cone_prearm_max_range_m,
                'bbox_padding_x_px': bbox_padding_x_px,
                'bbox_padding_y_px': bbox_padding_y_px,
                'max_detection_age_s': cone_entry_fusion_timeout_s,
                'cluster_rotation_deg': (
                    cone_entry_fusion_rotation_deg
                ),
                'projection_model': (
                    cone_entry_fusion_projection_model
                ),
            }],
        ),
        Node(
            package='mission_cone_drive',
            executable='lidar_dbscan_preprocessing_node',
            name='cone_fusion_preprocessing_node',
            output='screen',
            condition=IfCondition(PythonExpression([
                "'", start_lane_stack, "'.lower() == 'true' and '",
                enable_lane_pipeline, "'.lower() == 'true' and '",
                enable_scene_pipeline, "'.lower() == 'true' and '",
                enable_drive_control, "'.lower() == 'true' and '",
                cone_perception_mode, "' == 'lidar_dbscan'",
            ])),
            parameters=[{
                'scan_topic': '/scan_rotated',
                'raw_clusters_topic': '/clusters_raw',
                'clusters_topic': '/fusion_clusters',
                'enable_prearm_output': ParameterValue(
                    PythonExpression([
                        "'", cone_prearm_enabled, "'.lower() == 'true' and '",
                        cone_prearm_use_fusion, "'.lower() == 'true'",
                    ]),
                    value_type=bool,
                ),
                'prearm_raw_clusters_topic': '/prearm_clusters_raw',
                'require_green_signal': False,
                'min_range_m': lidar_dbscan_min_range_m,
                'max_range_m': lidar_dbscan_max_range_m,
                'dbscan_eps_m': lidar_dbscan_eps_m,
                'dbscan_min_samples': lidar_dbscan_min_samples,
                'max_cone_diameter_m': (
                    lidar_dbscan_max_cone_diameter_m
                ),
                'angle_bin_deg': lidar_dbscan_angle_bin_deg,
                'min_cluster_separation_m': (
                    lidar_dbscan_min_cluster_separation_m
                ),
                'prearm_min_range_m': lidar_dbscan_min_range_m,
                'prearm_max_range_m': cone_prearm_max_range_m,
                'prearm_dbscan_eps_m': cone_prearm_dbscan_eps_m,
                'prearm_dbscan_min_samples':
                    cone_prearm_dbscan_min_samples,
                'prearm_max_cone_diameter_m':
                    lidar_dbscan_max_cone_diameter_m,
                'prearm_angle_bin_deg': 0.0,
                'prearm_min_cluster_separation_m':
                    lidar_dbscan_min_cluster_separation_m,
            }],
        ),
        Node(
            package='mission_cone_drive',
            executable='preprocessing_node',
            name='preprocessing_node',
            output='screen',
            condition=IfCondition(PythonExpression([
                "'", start_lane_stack, "'.lower() == 'true' and '",
                enable_lane_pipeline, "'.lower() == 'true' and '",
                enable_scene_pipeline, "'.lower() == 'true' and '",
                enable_drive_control, "'.lower() == 'true' and '",
                cone_perception_mode, "' == 'lidar_dbscan'",
            ])),
            parameters=[{
                'scan_topic': '/scan_rotated',
                'clusters_topic': '/clusters',
                'require_green_signal': False,
            }],
        ),
        Node(
            package='mission_cone_drive',
            executable='lidar_cone_filter_node',
            name='lidar_cone_filter_node',
            output='screen',
            condition=IfCondition(PythonExpression([
                "'", start_lane_stack, "'.lower() == 'true' and '",
                enable_lane_pipeline, "'.lower() == 'true' and '",
                enable_scene_pipeline, "'.lower() == 'true' and '",
                enable_drive_control, "'.lower() == 'true' and '",
                cone_perception_mode, "' == 'lidar'",
            ])),
            parameters=[{
                'scan_topic': '/scan_rotated',
                'raw_clusters_topic': '/clusters_raw',
                'output_clusters_topic': '/clusters',
                'max_range_m': max_range_m,
                'x_max_m': x_max_m,
                'y_abs_max_m': y_abs_max_m,
                'cluster_min_points': lidar_cluster_min_points,
                'cluster_min_chord_m': lidar_cluster_min_chord_m,
                'cluster_max_radius_m': lidar_cluster_max_radius_m,
                'cluster_max_chord_m': lidar_cluster_max_chord_m,
                'gap_min_width_m': gap_min_width_m,
                'gap_max_width_m': gap_max_width_m,
                'gap_expected_width_m': gap_expected_width_m,
                'cone_spacing_m': cone_spacing_m,
                'pair_min_normal_alignment': (
                    cone_pair_min_normal_alignment
                ),
                'pair_max_longitudinal_m': (
                    cone_pair_max_longitudinal_m
                ),
                'midpoint_reference_gate_m': (
                    cone_midpoint_reference_gate_m
                ),
                'association_distance_m': (
                    lidar_association_distance_m
                ),
                'min_track_hits': lidar_min_track_hits,
                'max_track_misses': lidar_max_track_misses,
                'min_entry_pairs': lidar_min_entry_pairs,
                'min_pair_span_m': lidar_min_pair_span_m,
                'max_entry_width_spread_m': (
                    lidar_max_entry_width_spread_m
                ),
                'one_side_half_width_m': cone_one_side_half_width_m,
                'boundary_tolerance_m': lidar_boundary_tolerance_m,
                'min_one_side_points': lidar_min_one_side_points,
                'two_point_support_confirm_frames': (
                    lidar_two_point_support_confirm_frames
                ),
                'established_hold_frames': (
                    lidar_established_hold_frames
                ),
            }],
        ),
        Node(
            package='mission_cone_drive',
            executable='path_planning_node',
            name='path_planning_node',
            output='screen',
            condition=IfCondition(PythonExpression([
                "'", start_lane_stack, "'.lower() == 'true' and '",
                enable_lane_pipeline, "'.lower() == 'true' and '",
                enable_scene_pipeline, "'.lower() == 'true' and '",
                enable_drive_control, "'.lower() == 'true' and '",
                cone_perception_mode, "' == 'lidar'",
            ])),
            parameters=[{
                'clusters_topic': '/clusters',
                'path_topic': '/path',
                'require_green_signal': False,
                'use_precomputed_corridor': ParameterValue(
                    PythonExpression([
                        "'", cone_perception_mode, "' == 'lidar'",
                    ]),
                    value_type=bool,
                ),
                'x_max_m': x_max_m,
                'y_abs_max_m': y_abs_max_m,
                'gap_min_width_m': gap_min_width_m,
                'gap_max_width_m': gap_max_width_m,
                'gap_expected_width_m': gap_expected_width_m,
                'cone_spacing_m': cone_spacing_m,
                'one_side_half_width_m': cone_one_side_half_width_m,
                'pair_min_normal_alignment': (
                    cone_pair_min_normal_alignment
                ),
                'pair_max_longitudinal_m': (
                    cone_pair_max_longitudinal_m
                ),
                'midpoint_reference_gate_m': (
                    cone_midpoint_reference_gate_m
                ),
                'min_midpoints_for_path': cone_min_midpoints_for_path,
                'min_path_span_m': cone_min_path_span_m,
                'max_path_lateral_jump_m': cone_max_path_lateral_jump_m,
                'path_lost_keep_frames': cone_path_lost_keep_frames,
                'path_smoothing_alpha': cone_path_smoothing_alpha,
            }],
        ),
        Node(
            package='mission_cone_drive',
            executable='lidar_dbscan_path_planning_node',
            name='lidar_dbscan_path_planning_node',
            output='screen',
            condition=IfCondition(PythonExpression([
                "'", start_lane_stack, "'.lower() == 'true' and '",
                enable_lane_pipeline, "'.lower() == 'true' and '",
                enable_scene_pipeline, "'.lower() == 'true' and '",
                enable_drive_control, "'.lower() == 'true' and '",
                cone_perception_mode, "' == 'lidar_dbscan'",
            ])),
            parameters=[{
                'clusters_topic': '/clusters',
                'path_topic': '/path',
                'require_green_signal': False,
                'path_frame_id': 'rear_axle',
            }],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_dbscan_rear_axle_tf',
            output='screen',
            condition=IfCondition(PythonExpression([
                "'", start_lane_stack, "'.lower() == 'true' and '",
                enable_lane_pipeline, "'.lower() == 'true' and '",
                enable_scene_pipeline, "'.lower() == 'true' and '",
                enable_drive_control, "'.lower() == 'true' and '",
                cone_perception_mode, "' == 'lidar_dbscan'",
            ])),
            arguments=[
                '--x', '-0.42',
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
                "'", start_lane_stack, "'.lower() == 'true' and '",
                enable_lane_pipeline, "'.lower() == 'true' and '",
                enable_scene_pipeline, "'.lower() == 'true' and '",
                enable_drive_control, "'.lower() == 'true' and '",
                cone_controller, "' == 'pure_pursuit'",
            ])),
            parameters=[{
                'path_topic': '/path',
                'motor_topic': '/cone_motor_cmd',
                'require_green_signal': False,
                'min_speed': cone_min_speed,
                'max_speed': cone_max_speed,
            }],
        ),
        Node(
            package='mission_cone_drive',
            executable='cone_stanley_node',
            name='cone_stanley_node',
            output='screen',
            condition=IfCondition(PythonExpression([
                "'", start_lane_stack, "'.lower() == 'true' and '",
                enable_lane_pipeline, "'.lower() == 'true' and '",
                enable_scene_pipeline, "'.lower() == 'true' and '",
                enable_drive_control, "'.lower() == 'true' and '",
                cone_controller, "' == 'stanley'",
            ])),
            parameters=[{
                'path_topic': '/path',
                'motor_topic': '/cone_motor_cmd',
                'require_green_signal': False,
                'min_speed': cone_min_speed,
                'max_speed': cone_max_speed,
                'control_x_m': cone_stanley_control_x_m,
                'heading_window_m':
                    cone_stanley_heading_window_m,
                'cross_track_gain':
                    cone_stanley_cross_track_gain,
                'heading_gain': cone_stanley_heading_gain,
                'softening_speed_mps':
                    cone_stanley_softening_speed_mps,
                'max_angle_step': cone_stanley_max_angle_step,
                'smoothing_alpha':
                    cone_stanley_smoothing_alpha,
                'path_timeout_s': cone_stanley_path_timeout_s,
                'min_path_points': 2,
                'command_publish_rate_hz': 10.0,
            }],
        ),
        Node(
            package='mission_cone_drive',
            executable='mission_manager_node',
            name='mission_manager_node',
            output='screen',
            condition=IfCondition(lane_control_enabled),
            parameters=[{
                'lane_command_topic': '/lane_motor_cmd',
                'lane_stamped_command_topic': '/lane_motor_cmd_stamped',
                'start_service_name': mission_start_service_name,
                'stop_service_name': mission_stop_service_name,
                'pause_service_name': mission_pause_service_name,
                'resume_service_name': mission_resume_service_name,
                'toggle_pause_service_name':
                    mission_toggle_pause_service_name,
                'use_stamped_lane_command': True,
                'decision_rate_hz': mission_decision_rate_hz,
                'event_driven_lane_output': ParameterValue(
                    mission_event_driven_lane, value_type=bool),
                'lane_source_timeout_s': stanley_state_timeout_s,
                'future_stamp_tolerance_s': future_stamp_tolerance_s,
                'enable_scene_inputs': ParameterValue(
                    enable_scene_pipeline,
                    value_type=bool,
                ),
                'enable_cone_mission': ParameterValue(
                    enable_scene_pipeline,
                    value_type=bool,
                ),
                'cone_command_topic': '/cone_motor_cmd',
                'motor_topic': motor_topic,
                'detections_topic': '/scene_yolo/detections',
                'cone_perception_mode': cone_perception_mode,
                'cone_conf_threshold': cone_conf_threshold,
                'cone_entry_min_detections': cone_entry_min_detections,
                'entry_cone_clusters_topic': '/entry_cone_clusters',
                'prearm_cone_clusters_topic': '/cone_prearm_clusters',
                'cone_entry_min_clusters': ParameterValue(
                    PythonExpression([
                        lidar_entry_min_clusters,
                        " if '", cone_perception_mode,
                        "' == 'lidar' else ",
                        lidar_dbscan_entry_min_clusters,
                    ]),
                    value_type=int,
                ),
                'cone_entry_min_path_points': cone_entry_min_path_points,
                'cone_entry_use_fusion_gate': ParameterValue(
                    PythonExpression([
                        "'", start_yolo, "'.lower() == 'true' and '",
                        enable_scene_pipeline,
                        "'.lower() == 'true' and '",
                        cone_entry_use_fusion_gate,
                        "'.lower() == 'true' and '",
                        cone_perception_mode, "' == 'lidar_dbscan'",
                    ]),
                    value_type=bool,
                ),
                'cone_entry_allow_lidar_fallback':
                    cone_entry_allow_lidar_fallback,
                'cone_entry_min_fused_clusters':
                    cone_entry_min_fused_clusters,
                'cone_entry_require_both_sides':
                    cone_entry_require_both_sides,
                'cone_entry_side_min_abs_y_m':
                    cone_entry_side_min_abs_y_m,
                'cone_entry_fusion_timeout_s':
                    cone_entry_fusion_timeout_s,
                'cone_prearm_enabled': ParameterValue(
                    cone_prearm_enabled, value_type=bool),
                'cone_bbox_prearm_enabled': ParameterValue(
                    cone_bbox_prearm_enabled, value_type=bool),
                'cone_prearm_use_fusion': ParameterValue(
                    cone_prearm_use_fusion, value_type=bool),
                'cone_prearm_min_bbox_area_px':
                    cone_prearm_min_bbox_area_px,
                'cone_prearm_min_bottom_y_px':
                    cone_prearm_min_bottom_y_px,
                'cone_bbox_prearm_min_detections':
                    cone_bbox_prearm_min_detections,
                'cone_bbox_prearm_clear_frames':
                    cone_bbox_prearm_clear_frames,
                'cone_prearm_min_clusters': cone_prearm_min_clusters,
                'cone_prearm_window': cone_prearm_window,
                'cone_prearm_min_matches': cone_prearm_min_matches,
                'cone_prearm_hold_s': cone_prearm_hold_s,
                'cone_prearm_speed_cap': cone_prearm_speed_cap,
                'cone_suspect_min_fused_clusters':
                    cone_suspect_min_fused_clusters,
                'cone_suspect_hold_s': cone_suspect_hold_s,
                'cone_suspect_speed_cap': cone_suspect_speed_cap,
                'cone_allow_reentry': ParameterValue(
                    PythonExpression([
                        "'", lidar_allow_cone_reentry,
                        "' == 'true' if '", cone_perception_mode,
                        "' == 'lidar' else '",
                        lidar_dbscan_allow_cone_reentry,
                        "' == 'true'",
                    ]),
                    value_type=bool,
                ),
                'cone_enter_confirm_cycles': cone_enter_confirm_cycles,
                'cone_entry_min_bottom_y_px': cone_entry_min_bottom_y_px,
                'cone_min_mode_duration_s': cone_min_mode_duration_s,
                'cone_exit_hold_s': cone_exit_hold_s,
                'cone_speed_accel_rate_per_s':
                    cone_speed_accel_rate_per_s,
                'cone_speed_decel_rate_per_s':
                    cone_speed_decel_rate_per_s,
                'respect_traffic_light': ParameterValue(
                    enable_scene_pipeline,
                    value_type=bool,
                ),
                'start_active': start_active,
                'require_vesc_ready': ParameterValue(
                    start_motor_driver, value_type=bool),
                'vesc_ready_topic': '/vesc/ready',
                'vesc_ready_timeout_s': 1.25,
                'publish_pipeline_timing': ParameterValue(
                    publish_pipeline_timing, value_type=bool),
                'pipeline_timing_topic': pipeline_timing_topic,
            }],
        ),
        Node(
            package='mission_cone_drive',
            executable='keyboard_pause_node',
            name='integrated_keyboard_pause',
            output='screen',
            condition=IfCondition(enable_spacebar_pause),
            parameters=[{
                'toggle_service_name': mission_toggle_pause_service_name,
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
            name='integrated_drive_rviz',
            arguments=['-d', rviz_config],
            condition=IfCondition(start_rviz),
        ),
    ]
    global FORWARDED_LAUNCH_ARGUMENTS, NUMERIC_LAUNCH_ARGUMENT_TYPES
    FORWARDED_LAUNCH_ARGUMENTS = [
        action.name for action in actions
        if isinstance(action, DeclareLaunchArgument)
    ]
    NUMERIC_LAUNCH_ARGUMENT_TYPES = infer_numeric_argument_types(actions)
    if _needs_container_proxy():
        declarations = [
            action for action in actions
            if isinstance(action, DeclareLaunchArgument)
        ]
        return LaunchDescription([
            *declarations,
            OpaqueFunction(function=validate_configuration_only),
            Node(
                package='mission_cone_drive',
                executable='integrated_service_proxy',
                name='integrated_service_proxy',
                output='screen',
                parameters=[{
                    'container_label':
                        'com.xycar.role=integrated_drive',
                    'public_start_service': '/start_integrated_drive',
                    'public_stop_service': '/stop_integrated_drive',
                    'public_pause_service': '/pause_integrated_drive',
                    'public_resume_service': '/resume_integrated_drive',
                    'public_toggle_pause_service':
                        '/toggle_pause_integrated_drive',
                    'internal_start_service':
                        '/_start_integrated_drive_internal',
                    'internal_stop_service':
                        '/_stop_integrated_drive_internal',
                    'internal_pause_service':
                        '/_pause_integrated_drive_internal',
                    'internal_resume_service':
                        '/_resume_integrated_drive_internal',
                    'internal_toggle_pause_service':
                        '/_toggle_pause_integrated_drive_internal',
                    'enable_keyboard_pause': ParameterValue(
                        enable_spacebar_pause,
                        value_type=bool,
                    ),
                }],
            ),
            OpaqueFunction(function=launch_gpu_container),
        ])
    return LaunchDescription(actions)
