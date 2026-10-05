import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import launch
import xacro

def generate_launch_description():
    pkgPath= get_package_share_directory('cmgdevice_description')
    rvizConfigPath= os.path.join(pkgPath, 'config/config.rviz')

    urdf_file = os.path.join(pkgPath, 'urdf', 'cmgdevice.urdf.xacro')
    # urdf_file = os.path.join(pkgPath, 'urdf', 'foobar.urdf.xacro')

    # Read the URDF file contents into a string parameter
    #with open(urdf_file, 'r') as infp:
    #    robot_desc = infp.read()

    # Convert the xacro file into a plain XML URDF string
    doc = xacro.process_file(urdf_file)
    robot_desc = doc.toxml()


    params = {'robot_description': robot_desc }

    cmg_State_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[params],
        arguments=[urdf_file])
    
    cmg_Jointstate_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[params],
        arguments=[urdf_file],
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )

    cmg_Jointstate_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[params],
        arguments=[urdf_file],
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rvizConfigPath]
    )

 
    return launch.LaunchDescription( [
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                             description='This is a flag for joint state publisher gui'),
        cmg_State_publisher_node,
        cmg_Jointstate_publisher_node,
        cmg_Jointstate_publisher_gui_node,
        rviz_node
    ])
