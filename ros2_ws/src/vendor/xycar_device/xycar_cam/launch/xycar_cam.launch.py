from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    publish_camera_timing = LaunchConfiguration('publish_camera_timing')
    camera_timing_topic = LaunchConfiguration('camera_timing_topic')

    # Keep the camera profile here instead of relying on a usb_cam config file.
    # Jazzy installs params_1.yaml/params_2.yaml while older releases installed
    # params.yaml, so the old lookup silently left usb_cam on its YUYV defaults.
    return LaunchDescription([
        DeclareLaunchArgument(
            'publish_camera_timing',
            default_value='false',
        ),
        DeclareLaunchArgument(
            'camera_timing_topic',
            default_value='/camera_timing',
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='xycar_cam',
            arguments=['--ros-args', '--log-level', 'error'],
            parameters=[{
                'video_device': '/dev/video0',
                'framerate': 30.0,
                'io_method': 'mmap',
                'frame_id': 'default_cam',
                # The camera exposes MJPEG 640x480 at 120.101 FPS, while the
                # ROS driver samples it at 30 FPS.  With the driver's single
                # mmap buffer this keeps a fresh frame ready; native YUYV 30
                # FPS aliases with the 30 FPS timer and halves ROS output.
                'pixel_format': 'mjpeg2rgb',
                'av_device_format': 'YUV422P',
                'image_width': 640,
                'image_height': 480,
                'camera_name': 'default_cam',
                'camera_info_url':
                    'package://usb_cam/config/camera_info.yaml',
                'brightness': -1,
                'contrast': -1,
                'saturation': -1,
                'sharpness': -1,
                'gain': -1,
                'auto_white_balance': False,
                'white_balance': 4000,
                'autoexposure': False,
                'exposure': 100,
                'autofocus': False,
                'focus': -1,
                'publish_camera_timing': ParameterValue(
                    publish_camera_timing,
                    value_type=bool,
                ),
                'camera_timing_topic': camera_timing_topic,
            }],
        ),
    ])
