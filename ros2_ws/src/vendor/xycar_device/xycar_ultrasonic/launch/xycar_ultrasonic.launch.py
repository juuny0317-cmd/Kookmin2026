from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttySonic'),
        Node(
            package='xycar_ultrasonic',
            executable='xycar_ultrasonic',
            name='xycar_ultrasonic',
            output='screen',
            parameters=[{'port': LaunchConfiguration('port')}],
        ),
    ])
