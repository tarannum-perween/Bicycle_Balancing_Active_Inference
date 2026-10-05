import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    share = get_package_share_directory('bicycle')
    model_path = os.path.join(share, 'model', 'sim_main.urdf.xacro')
    world_path = os.path.join(share, 'model', 'test_world.world')

    robot_description = xacro.process_file(
        model_path,
        mappings={
            'add_rider': LaunchConfiguration('add_rider').perform(context),
            'rider_xyz': LaunchConfiguration('rider_xyz').perform(context),
        },
    ).toxml()

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={
            'gz_args': f"{LaunchConfiguration('gz_args').perform(context)} {world_path}",
        }.items(),
    )

    return [
        gz_launch,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }],
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-topic', 'robot_description',
                '-name', 'staebltech_simulation_platform',
                '-allow_renaming', 'true',
            ],
            condition=IfCondition(LaunchConfiguration('spawn')),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'add_rider',
            default_value='true',
            description='Include the fixed seated rider in the robot description.',
        ),
        DeclareLaunchArgument(
            'rider_xyz',
            default_value='0.29 0 1.01',
            description='Pelvis position relative to bicycle frame, metres.',
        ),
        DeclareLaunchArgument(
            'gz_args',
            default_value='-r',
            description='Arguments passed to Gazebo Sim. Use "-r -s" for server only.',
        ),
        DeclareLaunchArgument(
            'spawn',
            default_value='true',
            description='Spawn the bicycle into Gazebo Sim.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
