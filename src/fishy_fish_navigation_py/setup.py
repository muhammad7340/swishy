from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'fishy_fish_navigation_py'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='David Dudas',
    maintainer_email='david.dudas@outlook.com',
    description='Python nodes for slam, localization and navigation with Gazebo Harmonic and ROS Jazzy for Fish Robot ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'send_initialpose = fishy_fish_navigation_py.send_initialpose:main',
            'slam_toolbox_load_map = fishy_fish_navigation_py.slam_toolbox_load_map:main',
            'follow_waypoints = fishy_fish_navigation_py.follow_waypoints:main',
            'showcase_navigation = fishy_fish_navigation_py.showcase_navigation:main'
        ],
    },
)
