import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Locate your package share directory
    my_pkg_dir = get_package_share_directory('pid_cpp')

    # Locate the cmgdevice_description package share directory
    cmgdevice_description_dir = get_package_share_directory('cmgdevice_description')
    
    # 2. Path to the root YAML file that points to the configuration that will be used to
    # configure the CMG device used in the simulation.
    root_yaml_path = os.path.join(cmgdevice_description_dir, 'config', 'cmgparams.yaml')
    
    # 3. Parse the root YAML file during launch generation
    with open(root_yaml_path, 'r') as f:
        root_data = yaml.safe_load(f)
    
    # Extract the nested file name (navigating the ROS 2 YAML structure)
    # Note: '/**' or your specific node name namespace
    node_params = root_data.get('/**', {}).get('ros__parameters', {})
    sub_config_name = node_params.get('yaml_param_file', 'YAML FILE NOT SET!')  # Default fallback if not found

    if sub_config_name == 'YAML FILE NOT SET!':
        raise ValueError("The 'yaml_param_file' parameter is not set in the root YAML file.")
    
    # 4. Resolve the absolute path to the secondary configuration file
    sub_yaml_path = os.path.join(cmgdevice_description_dir, 'config', sub_config_name)
    
    # 5. Pass both parameter files to your ROS 2 Node
    executable_name = 'torque_Pgain_simple_ctl'  # Replace with your actual executable name
    my_node = Node(
        package='pid_cpp',
        executable=executable_name,
        name='pid_cpp_node',
        parameters=[
            root_yaml_path,  # Optional: includes 'config_file_name' in the node parameters
            sub_yaml_path   # Includes 'target_host' and 'target_port'
        ],
        output='screen'
    )
    
    return LaunchDescription([my_node])