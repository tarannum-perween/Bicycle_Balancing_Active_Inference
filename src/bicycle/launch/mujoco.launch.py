"""Launch the bicycle MJCF model in MuJoCo."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('bicycle')
    runner = os.path.join(share, 'scripts', 'run_mujoco.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value='bicycle.xml',
            description='MJCF filename in bicycle/model, or an absolute/relative path.',
        ),
        DeclareLaunchArgument(
            'duration',
            default_value='0',
            description='Seconds to simulate. Use 0 to run until the viewer closes.',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Set true to step the simulation without opening the viewer.',
        ),
        ExecuteProcess(
            cmd=[
                runner,
                '--model', LaunchConfiguration('model'),
                '--duration', LaunchConfiguration('duration'),
                '--headless',
                LaunchConfiguration('headless'),
            ],
            output='screen',
            shell=False,
        ),
    ])
