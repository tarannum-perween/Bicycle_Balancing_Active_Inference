"""Preview the reconstructed bicycle, optionally with its CMG, in RViz."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    share = get_package_share_directory('bicycle')
    model_path = os.path.join(share, 'model', LaunchConfiguration('model').perform(context))
    if model_path.endswith('.xacro'):
        description = ParameterValue(Command([
            'xacro ', model_path,
            ' add_rider:=', LaunchConfiguration('add_rider'),
            ' rider_xyz:="', LaunchConfiguration('rider_xyz'), '"',
        ]), value_type=str)
    else:
        with open(model_path, 'r', encoding='utf-8') as model_file:
            description = model_file.read()

    parameters = [{'robot_description': description}]
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=parameters, output='screen'),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             parameters=parameters, condition=UnlessCondition(LaunchConfiguration('gui'))),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             parameters=parameters, condition=IfCondition(LaunchConfiguration('gui'))),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(share, 'config', 'bicycle.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='sim_main.urdf.xacro',
                              description='Use bicycle.urdf.xacro for the bicycle alone'),
        DeclareLaunchArgument('add_rider', default_value='true',
                              description='Show the fixed seated human model'),
        DeclareLaunchArgument('rider_xyz', default_value='0.29 0 1.01',
                              description='Pelvis position relative to bicycle frame, metres'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
