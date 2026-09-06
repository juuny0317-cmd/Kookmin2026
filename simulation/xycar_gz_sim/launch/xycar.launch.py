"""Launch the measured Xycar on the existing Kookmin track or in real mode."""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml


def _as_bool(value: str) -> bool:
    return value.strip().lower() in ('true', '1', 'yes', 'on')


def _launch_setup(context):
    package_share = Path(get_package_share_directory('xycar_gz_sim'))
    config_path = package_share / 'config' / 'xycar.yaml'
    model_config_path = package_share / 'config' / 'model.yaml'
    with model_config_path.open(encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    mappings = {
        key: str(value).lower() if isinstance(value, bool) else str(value)
        for section in ('vehicle_model', 'sensor_model')
        for key, value in config[section].items()
        if key not in ('measured_nominal_max_front_wheel_rad',)
    }
    robot_description = xacro.process_file(
        str(package_share / 'urdf' / 'xycar.urdf.xacro'), mappings=mappings).toxml()

    use_sim_text = LaunchConfiguration('use_sim').perform(context)
    use_sim = _as_bool(use_sim_text)
    gui = _as_bool(LaunchConfiguration('gui').perform(context))
    world = LaunchConfiguration('world').perform(context)
    spawn_x = LaunchConfiguration('spawn_x').perform(context)
    spawn_y = LaunchConfiguration('spawn_y').perform(context)
    spawn_z = LaunchConfiguration('spawn_z').perform(context)
    spawn_yaw = LaunchConfiguration('spawn_yaw').perform(context)

    actions = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='xycar_robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim,
            }],
        ),
        Node(
            package='xycar_gz_sim',
            executable='command_adapter',
            name='command_adapter',
            output='screen',
            parameters=[str(config_path), {
                'use_sim': use_sim,
                'use_sim_time': use_sim,
            }],
        ),
    ]

    if not use_sim:
        return actions

    if not Path(world).is_file():
        raise FileNotFoundError(f'existing track world not found: {world}')

    supervisor = Path(get_package_prefix('xycar_gz_sim')) / 'lib' / 'xycar_gz_sim' / 'gz_supervisor'
    gz_command = [
        str(supervisor),
        '--world', world,
        '--server-config', str(package_share / 'config' / 'server.config'),
        '--version', '8',
    ]
    if gui:
        gz_command.extend(['--gui', '--gui-config', str(package_share / 'config' / 'gui.config')])
    gz_process = ExecuteProcess(
        cmd=gz_command,
        name='xycar_gz_supervisor',
        output='screen',
        sigterm_timeout='20',
        sigkill_timeout='5',
    )
    runtime_actions = [
        *actions,
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_xycar',
            output='screen',
            arguments=[
                '-world', 'kookmin_dxf_track',
                '-name', 'xycar',
                '-topic', 'robot_description',
                '-x', spawn_x, '-y', spawn_y, '-z', spawn_z, '-Y', spawn_yaw,
            ],
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='xycar_ros_gz_bridge',
            output='screen',
            arguments=[
                '/model/xycar/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/model/xycar/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/model/xycar/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                '/xycar_sim/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                '/xycar_sim/scan_raw@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            ],
            remappings=[
                ('/model/xycar/odometry', '/odom'),
                ('/model/xycar/tf', '/tf'),
            ],
            parameters=[{
                'qos_overrides./model/xycar/cmd_vel.subscriber.reliability': 'reliable',
                'qos_overrides./xycar_sim/image_raw.publisher.reliability': 'reliable',
                'qos_overrides./xycar_sim/scan_raw.publisher.reliability': 'best_effort',
            }],
        ),
        Node(
            package='xycar_gz_sim',
            executable='sensor_adapter',
            name='sensor_adapter',
            output='screen',
            parameters=[str(config_path), {'use_sim_time': True}],
        ),
    ]
    return [
        # Register shutdown handling before the process starts. Runtime nodes are
        # delayed so a rejected duplicate supervisor cannot send a spawn request
        # to the already-running server during launch shutdown.
        RegisterEventHandler(
            OnProcessExit(
                target_action=gz_process,
                on_exit=[EmitEvent(event=Shutdown(
                    reason='Gazebo supervisor exited; stopping the Xycar launch'))],
            )
        ),
        gz_process,
        TimerAction(period=1.0, actions=runtime_actions),
    ]


def generate_launch_description() -> LaunchDescription:
    default_world = str(
        Path(get_package_share_directory('xycar_gz_sim'))
        / 'worlds'
        / 'kookmin_track_from_dxf.world.sdf'
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='true', description='true: Gazebo, false: real /xycar_motor output'),
        DeclareLaunchArgument('gui', default_value='true', description='Start the Gazebo GUI'),
        DeclareLaunchArgument('world', default_value=default_world, description='Existing world; it is never modified'),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='-3.50'),
        DeclareLaunchArgument('spawn_z', default_value='0.02'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        OpaqueFunction(function=_launch_setup),
    ])
