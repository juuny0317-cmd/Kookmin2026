import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'cam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    package_data={
        package_name: ['*.pt'],
    },
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xytron',
    maintainer_email='xytron@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_publisher = cam.Camera_Publisher:main',
            'camera_timing_probe = cam.camera_timing_probe:main',
            'lane_detector = cam.Lane_Detector:main',
            'sign_detector = cam.SignDetector:main',
            'traffic_light_node = cam.traffic_light_node:main',
            'integrated_stanley_controller = cam.Integrated_Stanley_Controller:main',
            'yolo_node = cam.yolo_node:main',
            'frame_router = cam.frame_router:main',
            'centerlane_tracer = cam.centerlane_tracer:main',
            'resource_profiler = cam.resource_profiler:main',
            'topic_stamper = cam.topic_stamper:main',
            'target_lane_planner = cam.target_lane_planner:main',
            (
                'shortcut_left_turn_node = '
                'cam.shortcut_left_turn_node:main'
            ),
            'ultra_node = cam.ultra_node:main',
        ],
    },
)
