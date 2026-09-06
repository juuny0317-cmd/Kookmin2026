"""Launch one Xycar interface, straight-speed, or turning-radius test."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


TURN_CASES = {'turn_p20', 'turn_p30', 'turn_p40', 'turn_m20', 'turn_m30', 'turn_m40'}


def _setup(context):
    package_share = get_package_share_directory('xycar_gz_sim')
    test_case = LaunchConfiguration('test_case').perform(context)
    gui = LaunchConfiguration('gui').perform(context)
    world = LaunchConfiguration('world').perform(context)
    if test_case in TURN_CASES:
        spawn_x, spawn_y = '0.0', '0.0'
    else:
        spawn_x, spawn_y = '0.0', '-3.50'
    verification = Node(
        package='xycar_gz_sim',
        executable='verification',
        name='xycar_verification',
        output='screen',
        parameters=[
            os.path.join(package_share, 'config', 'xycar.yaml'),
            {'test_case': test_case, 'use_sim_time': True},
        ],
    )
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(package_share, 'launch', 'xycar.launch.py')),
            launch_arguments={
                'use_sim': 'true',
                'gui': gui,
                'world': world,
                'spawn_x': spawn_x,
                'spawn_y': spawn_y,
                'spawn_z': '0.02',
                'spawn_yaw': '0.0',
            }.items(),
        ),
        verification,
        RegisterEventHandler(
            OnProcessExit(
                target_action=verification,
                on_exit=[EmitEvent(event=Shutdown(reason=f'{test_case} verification completed'))],
            )
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    default_world = os.path.join(
        get_package_share_directory('xycar_gz_sim'),
        'worlds',
        'kookmin_track_from_dxf.world.sdf',
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'test_case',
            default_value='straight_zero',
            description=(
                'straight_zero, sensor_interface, speed_5, speed_10, speed_15, '
                'turn_p20/p30/p40, turn_m20/m30/m40')),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument(
            'world',
            default_value=default_world),
        OpaqueFunction(function=_setup),
    ])
