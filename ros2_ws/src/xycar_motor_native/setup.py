import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'xycar_motor_native'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xytron',
    maintainer_email='xytron@example.com',
    description='Native ROS 2 Xycar motor adapter and safe VESC launch.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'motor_command_adapter = '
            'xycar_motor_native.motor_command_adapter:main',
        ],
    },
)
