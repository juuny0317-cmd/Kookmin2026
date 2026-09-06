from glob import glob
from setuptools import find_packages, setup


package_name = 'xycar_gz_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml', 'README.md']),
        (f'share/{package_name}/config', glob('config/*')),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
        (f'share/{package_name}/urdf', glob('urdf/*')),
        (f'share/{package_name}/worlds', glob('../worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='planning_team',
    maintainer_email='planning@example.com',
    description='Xycar Gazebo Harmonic model and ROS 2 adapters',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'command_adapter = xycar_gz_sim.command_adapter_node:main',
            'gz_supervisor = xycar_gz_sim.gz_supervisor:main',
            'keyboard_teleop = xycar_gz_sim.keyboard_teleop_node:main',
            'sensor_adapter = xycar_gz_sim.sensor_adapter_node:main',
            'team_motor_adapter = xycar_gz_sim.team_motor_adapter_node:main',
            'verification = xycar_gz_sim.verification_node:main',
        ],
    },
)
