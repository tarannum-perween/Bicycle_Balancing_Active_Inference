import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node
import xacro

add_gazebo_gui = 'True'  # Global variable to hold the evaluated value
add_rider = 'False'  # Global variable to hold the evaluated value
add_joint_gui = 'False'  # Global variable to hold the evaluated value
add_PauseSimulation = 'True'  # Global variable to hold the evaluated value
add_rider = 'False'  # Global variable to hold the evaluated value

def evaluate_and_run(context, *args, **kwargs):
    global add_gazebo_gui, add_rider, add_joint_gui, add_PauseSimulation
    # 1. Use context.perform_substitution to get the real string value
    actual_joint_gui_value = context.perform_substitution(LaunchConfiguration('joint_gui'))
    joint_gui = context.perform_substitution(LaunchConfiguration('gazebo_gui'))
    add_rider = context.perform_substitution(LaunchConfiguration('add_rider'))

    # 2. Now you can print the actual string ('False' or 'True')
    print(f"add_rider: {add_rider}")
    print(f"joint_gui: {joint_gui}")
    print(f"gazebo_gui: {add_gazebo_gui}")
    add_PauseSimulation = context.perform_substitution(LaunchConfiguration('paused'))

    # Add nodes here that depend on if/else conditions based on the evaluated values!!

    return [] # Return any nodes or actions you want to launch here

def generate_launch_description():
    global add_gazebo_gui, add_joint_gui, add_PauseSimulation, add_rider
    #global add_rider

    # set some default values for the args for convenience:
    rider_arg = DeclareLaunchArgument(name='add_rider', default_value='False',
                                                      description='This is a flag for adding a rider on the bicycle')
    # joint_gui_arg = DeclareLaunchArgument(name='joint_gui', default_value='False',
    #                                                      description='This is a flag to manually alter the joint states')
    # gazebo_gui_arg = DeclareLaunchArgument(name='gazebo_gui', default_value='True',
    #                                                      description='This is a flag to enable the gazebo gui')
    # PauseSimulation_arg = DeclareLaunchArgument(name='paused', default_value='True',
    #                                                      description='This is a flag to pause the simulation')

    # # the simulation bicycle name defined within the bicycle xacro file
    # robotXacroName='staebltech_bicycle'

    # # this package name:
    namePackage='bicycle'

    LogInfo(msg=f'add_rider: {add_rider}, add_joint_gui: {add_joint_gui}, add_gazebo_gui: {add_gazebo_gui}, add_PauseSimulation: {add_PauseSimulation}')
    modelFileRelativePath = 'model/bicycle.urdf.xacro'
    if add_rider == False:
         #modelFileRelativePath = 'model/bicycle.urdf.xacro'
         modelFileRelativePath = 'model/sim_main.urdf.xacro'
    elif add_rider == True:
         modelFileRelativePath = 'model/bicycle_and_rider.urdf.xacro'

    worldFileRelativePath = 'model/test_world.world'
    basePath = get_package_share_directory(namePackage)

    pathModelFile = os.path.join(basePath, modelFileRelativePath)
    pathWorldFile = os.path.join(basePath, worldFileRelativePath)

    # get robot description 
    robotDescription = xacro.process_file(pathModelFile).toxml()
    
    # this is the launch file from the gazebo_ros package:
    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'),
                                                                             'launch','gazebo.launch.py'))
    # this is the gazebo launch description:
    gazeboLaunch = IncludeLaunchDescription(gazebo_rosPackageLaunch, launch_arguments={'world': pathWorldFile, 'paused': add_PauseSimulation, 'gui': add_gazebo_gui}.items())
    
    # create a gazebo_ros Node:
    spawnModelNode = Node(package='gazebo_ros', executable='spawn_entity.py', 
                              arguments=['-topic', 'robot_description', '-entity', robotXacroName],
                              output='screen')
    
    # Robot State Publisher Node
    nodeRobotStatePublisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robotDescription, 'use_sim_time': True}]
    )
    
    # # add the gazebo Nodes that will spawn gazebo and the devices/robots within the gazebo environment
    LaunchDescriptionObject = LaunchDescription()
    # LaunchDescriptionObject.add_action(gazeboLaunch)
    # LaunchDescriptionObject.add_action(spawnModelNode)
    # LaunchDescriptionObject.add_action(nodeRobotStatePublisher)
    # LaunchDescriptionObject.add_action(rider_arg)
    # LaunchDescriptionObject.add_action(joint_gui_arg)
    # LaunchDescriptionObject.add_action(gazebo_gui_arg)
    # LaunchDescriptionObject.add_action(PauseSimulation_arg)
    LaunchDescriptionObject.add_action(OpaqueFunction(function=evaluate_and_run))

    #LaunchDescriptionObject.get_launch_arguments( [rider_arg, joint_gui_arg, gazebo_gui_arg, PauseSimulation_arg, OpaqueFunction(function=evaluate_and_run)] )
    
    # add the nodes 
    return LaunchDescriptionObject
