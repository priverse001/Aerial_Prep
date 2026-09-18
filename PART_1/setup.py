import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'gps_denied_navigation'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='priverse',
    maintainer_email='user@todo.todo',
    description='Autonomous GPS-Denied Navigation & Vision Health System for PX4',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_bridge_health_node = gps_denied_navigation.vision_bridge_health_node:main',
            'offboard_position_hold_node = gps_denied_navigation.offboard_position_hold_node:main',
            'px4_sitl_odometry_bridge = gps_denied_navigation.px4_sitl_odometry_groundtruth_bridge:main',
            'px4_ekf2_configurator = gps_denied_navigation.px4_ekf2_param_configurator:main',
        ],
    },
)
