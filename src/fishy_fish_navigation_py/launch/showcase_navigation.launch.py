import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('fishy_fish_navigation_py')
    profiles = os.path.join(package_share, 'config', 'showcase_profiles.yaml')

    mode = LaunchConfiguration('mode')
    loop = LaunchConfiguration('loop')

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='showcase',
            choices=['showcase', 'explorer', 'patrol', 'dance', 'interactive'],
            description='Swishy navigation presentation mode',
        ),
        DeclareLaunchArgument(
            'loop',
            default_value='false',
            choices=['true', 'false'],
            description='Repeat finite presentation modes continuously',
        ),
        Node(
            package='fishy_fish_navigation_py',
            executable='showcase_navigation',
            name='showcase_navigation',
            output='screen',
            parameters=[
                profiles,
                {
                    'mode': mode,
                    'loop': ParameterValue(loop, value_type=bool),
                    'use_sim_time': True,
                },
            ],
        ),
    ])
